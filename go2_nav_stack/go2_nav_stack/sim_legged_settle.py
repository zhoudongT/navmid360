#!/usr/bin/env python3
"""四足 Gazebo 启动后持续发零速度，避免 CHAMP 站立阶段乱跳。"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SimLeggedSettle(Node):
    def __init__(self) -> None:
        super().__init__("sim_legged_settle")
        self.declare_parameter("settle_duration_sec", 10.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        topic = str(self.get_parameter("cmd_vel_topic").value)
        duration = float(self.get_parameter("settle_duration_sec").value)
        rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self._pub = self.create_publisher(Twist, topic, 10)
        self._zero = Twist()
        self._ticks = 0
        self._max_ticks = int(duration * rate_hz)
        self.create_timer(1.0 / rate_hz, self._on_tick)
        self.get_logger().info(
            f"四足站稳：向 {topic} 发零速度 {duration:.0f}s ({rate_hz:.0f} Hz)，"
            "请等 CHAMP 完成站立后再导航"
        )

    def _on_tick(self) -> None:
        if self._ticks >= self._max_ticks:
            return
        self._pub.publish(self._zero)
        self._ticks += 1
        if self._ticks >= self._max_ticks:
            self.get_logger().info("站稳阶段结束，交还 /cmd_vel 控制权")


def main() -> None:
    rclpy.init()
    node = SimLeggedSettle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
