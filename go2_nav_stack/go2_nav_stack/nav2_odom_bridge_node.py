#!/usr/bin/env python3
"""将 LIO + TF 合成 Nav2 使用的 /nav2_odom（odom->base_link 位姿 + LIO 速度）。"""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
from tf2_ros import TransformException


class Nav2OdomBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("nav2_odom_bridge")
        self.declare_parameter("input_topic", "/lio_odom_raw")
        self.declare_parameter("output_topic", "/nav2_odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 0.25)
        self.declare_parameter("republish_hz", 20.0)

        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self._last_twist = None
        self._last_lio_stamp = None

        self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(Odometry, out_topic, 10)
        self.create_subscription(Odometry, in_topic, self._on_odom, qos_profile_sensor_data)
        hz = float(self.get_parameter("republish_hz").value)
        if hz > 0.0:
            self.create_timer(1.0 / hz, self._on_timer)
        self.get_logger().info(
            f"nav2_odom_bridge: {in_topic} + TF -> {out_topic} ({self._base_frame}), {hz:.0f}Hz"
        )

    def _lookup_base_in_odom(self, stamp) -> tf2_ros.TransformStamped | None:
        timeout = rclpy.duration.Duration(seconds=self._tf_timeout)
        for t in (stamp, rclpy.time.Time()):
            try:
                return self._tf_buffer.lookup_transform(
                    self._odom_frame,
                    self._base_frame,
                    t,
                    timeout=timeout,
                )
            except TransformException:
                continue
        return None

    def _publish(self, stamp, tf) -> None:
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame
        out.pose.pose.position.x = tf.transform.translation.x
        out.pose.pose.position.y = tf.transform.translation.y
        out.pose.pose.position.z = tf.transform.translation.z
        out.pose.pose.orientation = tf.transform.rotation
        if self._last_twist is not None:
            out.twist = self._last_twist
        self._pub.publish(out)

    def _on_odom(self, msg: Odometry) -> None:
        self._last_twist = msg.twist
        self._last_lio_stamp = msg.header.stamp
        tf = self._lookup_base_in_odom(msg.header.stamp)
        if tf is None:
            self.get_logger().warn(
                f"无 TF {self._odom_frame}->{self._base_frame}，跳过 nav2 odom",
                throttle_duration_sec=5.0,
            )
            return
        self._publish(msg.header.stamp, tf)

    def _on_timer(self) -> None:
        if self._last_twist is None:
            return
        stamp = self._last_lio_stamp
        if stamp is None:
            stamp = self.get_clock().now().to_msg()
        tf = self._lookup_base_in_odom(stamp)
        if tf is None:
            return
        self._publish(stamp, tf)


def main() -> None:
    rclpy.init()
    node = Nav2OdomBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
