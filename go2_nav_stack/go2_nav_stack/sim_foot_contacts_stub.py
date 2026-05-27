#!/usr/bin/env python3
"""仿真足端接触占位：state_estimation 需 foot_contacts + joint_states 同步。"""

from __future__ import annotations

import rclpy
from champ_msgs.msg import ContactsStamped
from rclpy.node import Node


class SimFootContactsStub(Node):
    def __init__(self) -> None:
        super().__init__("sim_foot_contacts_stub")
        self.declare_parameter("topic", "foot_contacts")
        self.declare_parameter("publish_hz", 50.0)
        topic = str(self.get_parameter("topic").value)
        hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self._pub = self.create_publisher(ContactsStamped, topic, 10)
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(f"发布占位 {topic}（四足着地=true），供 CHAMP state_estimation")

    def _tick(self) -> None:
        msg = ContactsStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.contacts = [True, True, True, True]
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = SimFootContactsStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
