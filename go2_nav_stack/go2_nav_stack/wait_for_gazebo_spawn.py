#!/usr/bin/env python3
"""等待 Gazebo /spawn_entity 服务就绪后退出（供 launch 在 spawn 前调用）。"""

from __future__ import annotations

import sys

import rclpy
from gazebo_msgs.srv import SpawnEntity
from rclpy.node import Node


class WaitForGazeboSpawn(Node):
    def __init__(self) -> None:
        super().__init__("wait_for_gazebo_spawn")
        self.declare_parameter("timeout_sec", 120.0)
        self.declare_parameter("service_name", "/spawn_entity")
        timeout = float(self.get_parameter("timeout_sec").value)
        service = str(self.get_parameter("service_name").value).strip() or "/spawn_entity"

        client = self.create_client(SpawnEntity, service)
        self.get_logger().info(f"等待 {service}（最长 {timeout:.0f}s）…")
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f"超时：{service} 不可用。"
                "请确认 gzserver 已加载 libgazebo_ros_factory（使用 gazebo_ros 官方 gzserver.launch）。"
            )
            sys.exit(1)
        self.get_logger().info(f"{service} 已就绪，可 spawn 机器人")


def main() -> None:
    rclpy.init()
    node = WaitForGazeboSpawn()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
