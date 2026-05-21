#!/usr/bin/env python3
"""发布局部跟踪路径，供 RViz 显示。

- 全局路径：dong_path → /planned_path（map）
- 局部路径：MPPI 裁剪后的跟踪段 → /controller_server/transformed_global_plan
  本节点转发为 /local_path（map 系，便于与全局路径区分颜色）

仅在 FollowPath 执行中 MPPI 会持续更新 transformed_global_plan。
"""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import tf2_ros
from tf2_ros import TransformException


class LocalPathDisplayNode(Node):
    def __init__(self) -> None:
        super().__init__("local_path_display")
        self.declare_parameter(
            "input_topic", "/controller_server/transformed_global_plan"
        )
        self.declare_parameter("output_topic", "/local_path")
        self.declare_parameter("global_path_topic", "/planned_path")
        self.declare_parameter("fallback_near_robot_m", 5.0)
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("tf_timeout_sec", 0.15)

        self._out_topic = str(self.get_parameter("output_topic").value)
        self._global_topic = str(self.get_parameter("global_path_topic").value)
        self._near_m = float(self.get_parameter("fallback_near_robot_m").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self._got_mppi = False

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(Path, self._out_topic, qos)
        self._global_path: Path | None = None

        self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        in_topic = str(self.get_parameter("input_topic").value)
        self.create_subscription(Path, in_topic, self._on_mppi_path, qos)
        self.create_subscription(Path, self._global_topic, self._on_global, path_qos)
        self.create_timer(1.0, self._publish_fallback)

        self.get_logger().info(
            f"局部路径: {in_topic} -> {self._out_topic}；"
            f"未跟踪时从 {self._global_topic} 截取 {self._near_m:.0f}m 预览"
        )

    def _on_global(self, msg: Path) -> None:
        if msg.poses:
            self._global_path = msg

    def _on_mppi_path(self, msg: Path) -> None:
        if not msg.poses:
            return
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id or self._target_frame
        out.poses = list(msg.poses)
        self._pub.publish(out)
        if not self._got_mppi:
            self._got_mppi = True
            self.get_logger().info(
                f"MPPI 局部路径: {len(out.poses)} 点 frame={out.header.frame_id}"
            )

    def _robot_xy_in_frame(self, frame: str) -> tuple[float, float] | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                frame,
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except TransformException:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _publish_fallback(self) -> None:
        if self._got_mppi or self._global_path is None or not self._global_path.poses:
            return
        frame = self._global_path.header.frame_id or self._target_frame
        robot = self._robot_xy_in_frame(frame)
        if robot is None:
            return
        rx, ry = robot
        near: list = []
        for pose in self._global_path.poses:
            p = pose.pose.position
            if math.hypot(p.x - rx, p.y - ry) <= self._near_m:
                near.append(pose)
        if len(near) < 2:
            return
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = frame
        out.poses = near
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = LocalPathDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
