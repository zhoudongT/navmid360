#!/usr/bin/env python3
"""发布 MPPI 局部路径到 /local_path，供 RViz 显示（map 系）。

优先级:
  1. /controller_server/transformed_global_plan — MPPI 内部裁剪跟踪段（FollowPath 运行时）
  2. /global_path_processed — 走廊路径段（始终可用，作为预览/回退）
"""

from __future__ import annotations

import copy

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import TransformException


class LocalPathDisplayNode(Node):
    def __init__(self) -> None:
        super().__init__("local_path_display")
        self.declare_parameter(
            "input_topic", "/controller_server/transformed_global_plan"
        )
        self.declare_parameter(
            "input_topic_alt", "/transformed_global_plan"
        )
        self.declare_parameter("output_topic", "/local_path")
        self.declare_parameter("processed_path_topic", "/global_path_processed")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("tf_timeout_sec", 0.35)
        self.declare_parameter("mppi_stale_sec", 2.0)
        self.declare_parameter("publish_hz", 5.0)

        self._out_topic = str(self.get_parameter("output_topic").value)
        self._processed_topic = str(self.get_parameter("processed_path_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self._mppi_stale = float(self.get_parameter("mppi_stale_sec").value)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(Path, self._out_topic, qos)

        self._tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=30.0)
        )
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._processed_path: Path | None = None
        self._mppi_path: Path | None = None
        self._mppi_rx_time: Time | None = None
        self._last_pub_count = 0

        in_topic = str(self.get_parameter("input_topic").value)
        alt_topic = str(self.get_parameter("input_topic_alt").value)
        self.create_subscription(Path, in_topic, self._on_mppi_path, qos)
        if alt_topic and alt_topic != in_topic:
            self.create_subscription(Path, alt_topic, self._on_mppi_path, qos)
        self.create_subscription(
            Path, self._processed_topic, self._on_processed, path_qos
        )

        hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.create_timer(1.0 / hz, self._publish_timer)

        self.get_logger().info(
            f"局部路径 RViz: {in_topic} | {self._processed_topic} -> "
            f"{self._out_topic} (frame={self._target_frame})"
        )

    def _on_processed(self, msg: Path) -> None:
        if msg.poses:
            self._processed_path = msg

    def _on_mppi_path(self, msg: Path) -> None:
        if not msg.poses:
            return
        self._mppi_path = msg
        self._mppi_rx_time = self.get_clock().now()

    def _transform_path(self, path: Path) -> Path | None:
        src = path.header.frame_id or self._target_frame
        if not path.poses:
            return None
        if src == self._target_frame:
            out = Path()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self._target_frame
            out.poses = list(path.poses)
            return out
        try:
            tf = self._tf_buffer.lookup_transform(
                self._target_frame,
                src,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"路径 {src}->{self._target_frame} TF 失败: {exc}",
                throttle_duration_sec=8.0,
            )
            return None
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._target_frame
        for pose in path.poses:
            ps = PoseStamped()
            ps.header.frame_id = src
            ps.header.stamp = pose.header.stamp
            ps.pose = pose.pose
            try:
                tps = do_transform_pose(ps, tf)
            except Exception:
                continue
            out_pose = copy.deepcopy(pose)
            out_pose.header.frame_id = self._target_frame
            out_pose.header.stamp = out.header.stamp
            out_pose.pose = tps.pose
            out.poses.append(out_pose)
        return out if len(out.poses) >= 2 else None

    def _mppi_fresh(self) -> bool:
        if self._mppi_path is None or self._mppi_rx_time is None:
            return False
        age = (self.get_clock().now() - self._mppi_rx_time).nanoseconds * 1e-9
        return age <= self._mppi_stale

    def _publish_timer(self) -> None:
        src_path: Path | None = None
        source = "none"
        if self._mppi_fresh() and self._mppi_path is not None:
            src_path = self._mppi_path
            source = "mppi"
        elif self._processed_path is not None:
            src_path = self._processed_path
            source = "corridor"

        if src_path is None:
            return
        out = self._transform_path(src_path)
        if out is None:
            return
        self._pub.publish(out)
        n = len(out.poses)
        if n != self._last_pub_count or source == "mppi":
            self.get_logger().info(
                f"/local_path: {n} 点 source={source} "
                f"(MPPI={'fresh' if source == 'mppi' else 'waiting'})",
                throttle_duration_sec=4.0,
            )
        self._last_pub_count = n


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
