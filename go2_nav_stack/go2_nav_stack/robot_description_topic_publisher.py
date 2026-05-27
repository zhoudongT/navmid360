#!/usr/bin/env python3
"""向 /robot_description 发布 URDF；可选同时给 robot_state_publisher 用 xacro 路径。"""

from __future__ import annotations

import subprocess
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class RobotDescriptionTopicPublisher(Node):
    def __init__(self) -> None:
        super().__init__("robot_description_topic_publisher")
        self.declare_parameter("xacro_path", "")
        path = str(self.get_parameter("xacro_path").value).strip()
        if not path:
            raise RuntimeError("xacro_path 未设置")
        self.get_logger().info(f"xacro: {path}")
        try:
            urdf = subprocess.check_output(["xacro", path], text=True)
        except subprocess.CalledProcessError as exc:
            self.get_logger().error(f"xacro 失败: {exc}")
            sys.exit(1)
        if not urdf.strip():
            raise RuntimeError(f"xacro 输出为空: {path}")

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(String, "/robot_description", qos)
        self._msg = String(data=urdf)
        self._pub.publish(self._msg)
        self.get_logger().info(
            f"已发布 /robot_description（{len(urdf)} 字节），可供 Gazebo spawn"
        )
        self.create_timer(5.0, self._republish)

    def _republish(self) -> None:
        self._pub.publish(self._msg)


def main() -> None:
    rclpy.init()
    node = RobotDescriptionTopicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
