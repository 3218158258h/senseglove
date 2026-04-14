#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from senseglove_msgs.msg import SenseGloveState


class CycloneDDSHandStateSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("cyclonedds_hand_state_subscriber")

        self.declare_parameter("topic", "senseglove_states")
        self.declare_parameter("print_hz", 10.0)
        self.declare_parameter("show_all_hand_joints", False)

        topic = self.get_parameter("topic").get_parameter_value().string_value
        print_hz = self.get_parameter("print_hz").get_parameter_value().double_value
        self._show_all_hand_joints = (
            self.get_parameter("show_all_hand_joints").get_parameter_value().bool_value
        )

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
            f"Listening on topic '{topic}' (set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp)."
        )

    def _state_callback(self, msg: SenseGloveState) -> None:
        self._latest_msg = msg

    @staticmethod
    def _fmt_xyz(v) -> str:
        return f"({v.x:+.4f}, {v.y:+.4f}, {v.z:+.4f})"

    def _print_latest_state(self) -> None:
        if self._latest_msg is None:
            return

        msg = self._latest_msg

        lines = []
        lines.append(
            f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} frame={msg.header.frame_id}"
        )

        if msg.finger_tip_position:
            finger_names = ["thumb", "index", "middle", "ring", "little"]
            tip_parts = []
            for i, p in enumerate(msg.finger_tip_position):
                label = finger_names[i] if i < len(finger_names) else f"tip{i}"
                tip_parts.append(f"{label}:{self._fmt_xyz(p)}")
            lines.append("fingertips " + " | ".join(tip_parts))

        if msg.hand_position:
            if self._show_all_hand_joints:
                joint_parts = [
                    f"j{i}:{self._fmt_xyz(p)}" for i, p in enumerate(msg.hand_position)
                ]
                lines.append("hand_joints " + " | ".join(joint_parts))
            else:
                key_indices = [0, 3, 4, 7, 8, 11, 12, 15, 16, 19]
                sampled = []
                for idx in key_indices:
                    if idx < len(msg.hand_position):
                        sampled.append(f"j{idx}:{self._fmt_xyz(msg.hand_position[idx])}")
                if sampled:
                    lines.append("key_bones " + " | ".join(sampled))

        if msg.joint_names and msg.position:
            pos_items = [
                f"{name}:{float(pos):+.3f}"
                for name, pos in zip(msg.joint_names, msg.position)
            ]
            lines.append("joint_pos(rad) " + " | ".join(pos_items))

        if msg.joint_names and msg.absolute_velocity:
            vel_items = [
                f"{name}:{float(vel):+.3f}"
                for name, vel in zip(msg.joint_names, msg.absolute_velocity)
            ]
            lines.append("joint_vel(abs) " + " | ".join(vel_items))

        self.get_logger().info(" || ".join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CycloneDDSHandStateSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
