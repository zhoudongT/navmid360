#!/usr/bin/env python3
"""将 KISS-ICP 6DoF 里程计约束为地面四足平面：z 固定、仅保留 yaw。"""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    h = yaw * 0.5
    return (0.0, 0.0, math.sin(h), math.cos(h))


def _normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class PlanarOdomFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("planar_odom_filter_node")
        self.declare_parameter("input_topic", "/kiss_odom_raw")
        self.declare_parameter("output_topic", "/state_estimation")
        self.declare_parameter("lock_z", True)
        self.declare_parameter("z_fixed", 0.0)
        self.declare_parameter("planar_orientation", True)
        self.declare_parameter("yaw_smooth_alpha", 0.25)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self._lock_z = bool(self.get_parameter("lock_z").value)
        self._z_fixed = float(self.get_parameter("z_fixed").value)
        self._planar = bool(self.get_parameter("planar_orientation").value)
        self._yaw_alpha = float(self.get_parameter("yaw_smooth_alpha").value)
        self._yaw_filt: float | None = None

        self._pub = self.create_publisher(Odometry, out_topic, 20)
        self.create_subscription(Odometry, in_topic, self._on_odom, 20)
        self.get_logger().info(f"{in_topic} -> {out_topic} (planar odom)")

    def _on_odom(self, msg: Odometry) -> None:
        out = Odometry()
        # 与 tf_manager 发布的 TF 时间戳对齐，避免 RViz Odometry 显示 queue is full
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = msg.pose.pose.position.x
        out.pose.pose.position.y = msg.pose.pose.position.y
        out.pose.pose.position.z = (
            self._z_fixed if self._lock_z else msg.pose.pose.position.z
        )
        o = msg.pose.pose.orientation
        yaw = _yaw_from_quat(o.x, o.y, o.z, o.w)
        if self._yaw_alpha > 0.0:
            if self._yaw_filt is None:
                self._yaw_filt = yaw
            else:
                dy = _normalize_angle(yaw - self._yaw_filt)
                self._yaw_filt = _normalize_angle(
                    self._yaw_filt + self._yaw_alpha * dy
                )
            yaw = self._yaw_filt
        if self._planar:
            qx, qy, qz, qw = _quat_from_yaw(yaw)
            out.pose.pose.orientation.x = qx
            out.pose.pose.orientation.y = qy
            out.pose.pose.orientation.z = qz
            out.pose.pose.orientation.w = qw
        else:
            out.pose.pose.orientation = o
        out.pose.covariance = msg.pose.covariance
        out.twist = msg.twist
        if self._planar:
            out.twist.twist.linear.z = 0.0
            out.twist.twist.angular.x = 0.0
            out.twist.twist.angular.y = 0.0
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = PlanarOdomFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
