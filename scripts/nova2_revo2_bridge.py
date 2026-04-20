#!/usr/bin/env python3
"""
Bridge: SenseGlove Nova2 → Revo2 Dexterous Hand (dual hand)

ROS2 topics consumed:
  RH: /senseglove/glove<RIGHT_GLOVE_SERIAL>/rh/nova2_normalized_input
  LH: /senseglove/glove<LEFT_GLOVE_SERIAL>/lh/nova2_normalized_input
  (type: std_msgs/Float32MultiArray, 6 values, range 0-1)

nova2_normalized_input index → sensor mapping:
  [0]  Thumb Abduction        (内收/外展)
  [1]  Thumb FlexionProximal  (拇指屈伸)
  [2]  Index FlexionProximal
  [3]  Index FlexionDistal    ← used for index finger (食指只用第二个)
  [4]  Middle Flexion
  [5]  Ring Flexion           ← shared with pinky (无名指/小拇指共用)

Revo2 6-DOF positions (0 = fist/closed, 1000 = fully open):
  [0]  大拇指Flex  ← nova2[1] inverted  (1=flexed → 0,  0=open → 1000)
  [1]  大拇指Aux   ← nova2[0] direct    (0=adducted → 0, 1=spread → 1000)
  [2]  食指        ← nova2[3] inverted
  [3]  中指        ← nova2[4] inverted
  [4]  无名指      ← nova2[5] inverted
  [5]  小拇指      ← nova2[5] inverted  (same source as 无名指)

Hardware connections:
  LEFT_PORT  → left  hand (glove serial LEFT_GLOVE_SERIAL,  Modbus ID LEFT_ID)
  RIGHT_PORT → right hand (glove serial RIGHT_GLOVE_SERIAL, Modbus ID RIGHT_ID)
"""

import asyncio
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

from revo2_utils import logger, libstark
from utils import setup_shutdown_event

# ─── Configuration ────────────────────────────────────────────────────────────
# Glove serial numbers visible in the ROS topic namespace
RIGHT_GLOVE_SERIAL = "00726"
LEFT_GLOVE_SERIAL = "00733"

# Serial ports for the Revo2 hands
RIGHT_PORT = "/dev/ttyUSB0"
LEFT_PORT = "/dev/ttyUSB1"

# Modbus device IDs
RIGHT_ID = 0x7F
LEFT_ID = 0x7E

BAUDRATE = libstark.Baudrate.Baud460800

# How often to send position commands to the Revo2 hands (Hz)
CONTROL_HZ = 20

# Motor speeds (max = 1000)
SPEEDS = [1000] * 6


# ─── Sensor → Revo2 conversion ────────────────────────────────────────────────
def nova2_to_revo2(values: list[float]) -> list[int]:
    """Convert 6 nova2 normalised sensor values (0-1) to Revo2 positions (0-1000).

    Nova2 flexion convention:   0 = straight/open,  1 = fully flexed/closed
    Revo2 position convention:  0 = fist/closed,    1000 = fully open

    So flexion channels are inverted: revo2 = (1 - nova2) * 1000
    Abduction is direct:              revo2 = nova2 * 1000
    """

    def inv(v: float) -> int:
        """Inverted mapping: open sensor → high position."""
        return int(max(0.0, min(1.0, 1.0 - v)) * 1000)

    def fwd(v: float) -> int:
        """Direct mapping: large sensor value → high position."""
        return int(max(0.0, min(1.0, v)) * 1000)

    if len(values) < 6:
        return [0] * 6

    return [
        inv(values[1]),  # [0] 大拇指Flex  ← Thumb FlexionProximal (inverted)
        fwd(values[0]),  # [1] 大拇指Aux   ← Thumb Abduction       (direct)
        inv(values[3]),  # [2] 食指        ← Index FlexionDistal   (inverted)
        inv(values[4]),  # [3] 中指        ← Middle Flexion        (inverted)
        inv(values[5]),  # [4] 无名指      ← Ring Flexion          (inverted)
        inv(values[5]),  # [5] 小拇指      ← Ring Flexion          (inverted, same)
    ]


# ─── ROS2 subscriber node ─────────────────────────────────────────────────────
class Nova2BridgeNode(Node):
    """Subscribes to both gloves' nova2_normalized_input topics and stores the latest values."""

    def __init__(self) -> None:
        super().__init__("nova2_revo2_bridge")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        rh_topic = f"/senseglove/glove{RIGHT_GLOVE_SERIAL}/rh/nova2_normalized_input"
        lh_topic = f"/senseglove/glove{LEFT_GLOVE_SERIAL}/lh/nova2_normalized_input"

        # Latest raw sensor values for each hand (6 floats, 0-1)
        self._rh_values: list[float] = [0.0] * 6
        self._lh_values: list[float] = [0.0] * 6

        self.create_subscription(Float32MultiArray, rh_topic, self._rh_cb, qos)
        self.create_subscription(Float32MultiArray, lh_topic, self._lh_cb, qos)

        self.get_logger().info(f"Subscribed RH: {rh_topic}")
        self.get_logger().info(f"Subscribed LH: {lh_topic}")

    def _rh_cb(self, msg: Float32MultiArray) -> None:
        self._rh_values = list(msg.data)

    def _lh_cb(self, msg: Float32MultiArray) -> None:
        self._lh_values = list(msg.data)

    def get_rh_positions(self) -> list[int]:
        """Return current right-hand Revo2 target positions (0-1000)."""
        return nova2_to_revo2(self._rh_values)

    def get_lh_positions(self) -> list[int]:
        """Return current left-hand Revo2 target positions (0-1000)."""
        return nova2_to_revo2(self._lh_values)


# ─── Revo2 async control loop ─────────────────────────────────────────────────
async def control_loop(
    client_left,
    client_right,
    node: Nova2BridgeNode,
    shutdown_event: asyncio.Event,
) -> None:
    interval = 1.0 / CONTROL_HZ
    logger.info("Control loop started — sending positions at %d Hz", CONTROL_HZ)

    while not shutdown_event.is_set():
        rh_pos = node.get_rh_positions()
        lh_pos = node.get_lh_positions()

        logger.debug(
            "RH %s  LH %s",
            [f"{v:4d}" for v in rh_pos],
            [f"{v:4d}" for v in lh_pos],
        )

        try:
            await asyncio.gather(
                client_right.set_finger_positions_and_speeds(RIGHT_ID, rh_pos, SPEEDS),
                client_left.set_finger_positions_and_speeds(LEFT_ID, lh_pos, SPEEDS),
            )
        except Exception as exc:  # noqa: BLE001
            logger.error("Error sending positions: %s", exc)

        await asyncio.sleep(interval)


async def main_async(node: Nova2BridgeNode) -> None:
    libstark.init_logging()
    shutdown_event = setup_shutdown_event(logger)

    client_left = await libstark.modbus_open(LEFT_PORT, BAUDRATE)
    client_right = await libstark.modbus_open(RIGHT_PORT, BAUDRATE)
    if not client_left or not client_right:
        logger.critical("Failed to open one or both serial ports.")
        sys.exit(1)

    left_info = await client_left.get_device_info(LEFT_ID)
    right_info = await client_right.get_device_info(RIGHT_ID)
    if not left_info or not right_info:
        logger.critical("Failed to get device info for one or both hands.")
        sys.exit(1)

    logger.info("Left  hand: %s", left_info.description)
    logger.info("Right hand: %s", right_info.description)

    task = asyncio.create_task(
        control_loop(client_left, client_right, node, shutdown_event)
    )

    await shutdown_event.wait()

    task.cancel()
    await asyncio.gather(task, return_exceptions=True)
    await client_left.modbus_close()
    await client_right.modbus_close()
    logger.info("Modbus clients closed")


# ─── Entry point ──────────────────────────────────────────────────────────────
def main() -> None:
    rclpy.init()
    node = Nova2BridgeNode()

    # Spin ROS2 in a background thread so asyncio can own the main thread.
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        asyncio.run(main_async(node))
    except KeyboardInterrupt:
        logger.info("User interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=2.0)

    sys.exit(0)


if __name__ == "__main__":
    main()
