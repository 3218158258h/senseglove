#!/usr/bin/env python3

from __future__ import annotations

from typing import Iterable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from senseglove_msgs.msg import SenseGloveState

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:
    class ExternalShutdownException(Exception):
        """Compatibility fallback when rclpy lacks ExternalShutdownException."""


class CycloneDDSHandStateSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("cyclonedds_hand_state_subscriber")

        self.declare_parameter("topic", "")
        self.declare_parameter("glove_serial", "")
        self.declare_parameter("hand", "rh")
        self.declare_parameter("print_hz", 10.0)
        self.declare_parameter("show_all_hand_joints", False)

        topic_param = self.get_parameter("topic").get_parameter_value().string_value.strip()
        glove_serial = (
            self.get_parameter("glove_serial").get_parameter_value().string_value.strip()
        )
        hand = self.get_parameter("hand").get_parameter_value().string_value.strip().lower()
        print_hz = self.get_parameter("print_hz").get_parameter_value().double_value
        self._show_all_hand_joints = (
            self.get_parameter("show_all_hand_joints").get_parameter_value().bool_value
        )

        if hand not in {"rh", "lh"}:
            self.get_logger().warning(
                f"Invalid hand='{hand}', expected 'rh' or 'lh'. Falling back to 'rh'."
            )
            hand = "rh"

        if topic_param:
            topic = topic_param
        elif glove_serial:
            topic = f"/senseglove/glove{glove_serial}/{hand}/senseglove_states"
        else:
            topic = "senseglove_states"

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._latest_msg: Optional[SenseGloveState] = None
        self.create_subscription(
            SenseGloveState, topic, self._state_callback, qos_profile
        )

        safe_hz = max(print_hz, 0.1)
        self.create_timer(1.0 / safe_hz, self._print_latest_state)
        self.get_logger().info(
            f"Listening on topic '{topic}' "
            f"(set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp, hand={hand}, glove_serial='{glove_serial}')."
        )

    def _state_callback(self, msg: SenseGloveState) -> None:
        self._latest_msg = msg

    @staticmethod
    def _fmt_xyz(v) -> str:
        return f"({v.x:+.4f}, {v.y:+.4f}, {v.z:+.4f})"

    @staticmethod
    def _chunk(items: list[str], chunk_size: int) -> Iterable[list[str]]:
        for i in range(0, len(items), chunk_size):
            yield items[i : i + chunk_size]

    def _append_section(
        self, lines: list[str], title: str, items: list[str], items_per_line: int
    ) -> None:
        if not items:
            return
        lines.append(f"{title}:")
        for row in self._chunk(items, items_per_line):
            lines.append("  " + " | ".join(row))

    def _print_latest_state(self) -> None:
        if self._latest_msg is None:
            return

        msg = self._latest_msg

        lines: list[str] = [
            f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} frame={msg.header.frame_id}"
        ]

        if msg.finger_tip_position:
            finger_names = ["thumb", "index", "middle", "ring", "little"]
            tip_parts = []
            for i, p in enumerate(msg.finger_tip_position):
                label = finger_names[i] if i < len(finger_names) else f"tip{i}"
                tip_parts.append(f"{label}={self._fmt_xyz(p)}")
            self._append_section(lines, "fingertips", tip_parts, 2)

        if msg.hand_position:
            if self._show_all_hand_joints:
                joint_parts = [
                    f"j{i}={self._fmt_xyz(p)}" for i, p in enumerate(msg.hand_position)
                ]
                self._append_section(lines, "hand_joints", joint_parts, 2)
            else:
                key_indices = [0, 3, 4, 7, 8, 11, 12, 15, 16, 19]
                sampled = []
                for idx in key_indices:
                    if idx < len(msg.hand_position):
                        sampled.append(f"j{idx}={self._fmt_xyz(msg.hand_position[idx])}")
                self._append_section(lines, "key_bones", sampled, 2)

        if msg.joint_names and msg.position:
            pos_items = [
                f"{name}={float(pos):+.3f}"
                for name, pos in zip(msg.joint_names, msg.position)
            ]
            self._append_section(lines, "joint_pos(rad)", pos_items, 4)

        if msg.joint_names and msg.absolute_velocity:
            vel_items = [
                f"{name}={float(vel):+.3f}"
                for name, vel in zip(msg.joint_names, msg.absolute_velocity)
            ]
            self._append_section(lines, "joint_vel(abs)", vel_items, 4)

        self.get_logger().info("\n" + "\n".join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CycloneDDSHandStateSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
