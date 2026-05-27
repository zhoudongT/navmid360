#!/usr/bin/env python3
"""仿真启动后自动发布一次 /initialpose，解除 ICP「必须手动点姿态」死锁。"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node


def _yaw_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class SimInitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("sim_initialpose_publisher")
        self.declare_parameter("delay_sec", 12.0)
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("frame_id", "map")
        self._pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._sent = False
        delay = float(self.get_parameter("delay_sec").value)
        self.create_timer(delay, self._publish_once)
        self.get_logger().info(
            f"将在 {delay:.0f}s 后发布 /initialpose "
            f"({self.get_parameter('x').value}, {self.get_parameter('y').value}, "
            f"yaw={self.get_parameter('yaw').value})"
        )

    def _publish_once(self) -> None:
        if self._sent:
            return
        self._sent = True
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.pose.pose.position.x = float(self.get_parameter("x").value)
        msg.pose.pose.position.y = float(self.get_parameter("y").value)
        msg.pose.pose.position.z = float(self.get_parameter("z").value)
        msg.pose.pose.orientation = _yaw_quat(float(self.get_parameter("yaw").value))
        self._pub.publish(msg)
        self.get_logger().info("已发布 /initialpose（仿真种子位姿）")


def main() -> None:
    rclpy.init()
    node = SimInitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
