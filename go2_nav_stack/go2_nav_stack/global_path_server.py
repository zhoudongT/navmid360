#!/usr/bin/env python3
"""全局路径状态（对接 dong_path /planned_path，单地图不切图）。"""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String


class GlobalPathServer(Node):
    def __init__(self) -> None:
        super().__init__("global_path_server")
        self.declare_parameter("planned_path_topic", "/planned_path")
        self._status_pub = self.create_publisher(String, "/global_path/status", 10)
        self._have_path = False
        topic = self.get_parameter("planned_path_topic").value
        self.create_subscription(Path, topic, self._on_path, 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f"global_path_server: subscribe {topic}")

    def _on_path(self, msg: Path) -> None:
        self._have_path = len(msg.poses) > 0
        n = len(msg.poses)
        if n:
            self._status_pub.publish(String(data=f"ACTIVE poses={n} frame={msg.header.frame_id}"))

    def _tick(self) -> None:
        if not self._have_path:
            self._status_pub.publish(String(data="IDLE"))


def main() -> None:
    rclpy.init()
    node = GlobalPathServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
