#!/usr/bin/env python3
"""将 Go2 导航栈话题桥接到 ego-planner-swarm（drone_0_* 命名）。"""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros
from tf2_ros import TransformException


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


class EgoGo2BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("ego_go2_bridge")
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("publish_state_estimation", False)
        self.declare_parameter("odom_input_topic", "/lio_odom_raw")
        self.declare_parameter("cloud_input_topic", "/cloud_registered_map")
        self.declare_parameter("cloud_fallback_topic", "/transformed_cloud")
        self.declare_parameter("cloud_fallback_stale_sec", 0.45)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 0.3)
        self.declare_parameter("cloud_stale_warn_sec", 0.35)
        # 相对 base_link：激光下 30cm 至向上高度带内的点作为 EGO 障碍（可通行区外）
        self.declare_parameter("cloud_crop_in_base_frame", True)
        self.declare_parameter("lidar_height_m", 0.27)
        self.declare_parameter("obstacle_below_lidar_m", 0.30)
        self.declare_parameter("obstacle_max_height_above_base_m", 1.0)
        self.declare_parameter("enable_cloud_z_crop", True)
        # map 系绝对高度裁剪（仅 cloud_crop_in_base_frame=false 时使用）
        self.declare_parameter("cloud_z_min_m", 0.05)
        self.declare_parameter("cloud_z_max_m", 0.55)

        did = int(self.get_parameter("drone_id").value)
        self._crop_base = bool(self.get_parameter("cloud_crop_in_base_frame").value)
        self._lidar_h = float(self.get_parameter("lidar_height_m").value)
        self._below_lidar = float(self.get_parameter("obstacle_below_lidar_m").value)
        self._max_h_base = float(
            self.get_parameter("obstacle_max_height_above_base_m").value
        )
        self._z_min_base = self._lidar_h - self._below_lidar
        self._cloud_z_min = float(self.get_parameter("cloud_z_min_m").value)
        self._cloud_z_max = float(self.get_parameter("cloud_z_max_m").value)
        self._crop_cloud = bool(self.get_parameter("enable_cloud_z_crop").value)
        prefix = f"/drone_{did}_"
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)

        self._publish_odom = bool(self.get_parameter("publish_state_estimation").value)
        self._odom_pub = None
        if self._publish_odom:
            self._odom_pub = self.create_publisher(
                Odometry, f"{prefix}state_estimation", 20
            )

        self._cloud_pub = self.create_publisher(
            PointCloud2, f"{prefix}cloud_registered_map", 10
        )

        self._buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._listener = tf2_ros.TransformListener(self._buffer, self)

        in_odom = str(self.get_parameter("odom_input_topic").value)
        in_cloud = str(self.get_parameter("cloud_input_topic").value)
        fallback_cloud = str(self.get_parameter("cloud_fallback_topic").value)
        self._cloud_fallback_stale = float(
            self.get_parameter("cloud_fallback_stale_sec").value
        )
        if self._publish_odom:
            self.create_subscription(Odometry, in_odom, self._on_odom, 20)
        self.create_subscription(PointCloud2, in_cloud, self._on_cloud_primary, 10)
        if fallback_cloud:
            self.create_subscription(
                PointCloud2, fallback_cloud, self._on_cloud_fallback, 10
            )

        self._last_cloud_rx_sec = 0.0
        self._last_primary_cloud_rx_sec = 0.0
        self.create_timer(0.5, self._check_cloud_freshness)

        crop_desc = (
            f"base z∈[{self._z_min_base:.2f},{self._max_h_base:.2f}]m "
            f"(激光{self._lidar_h}m下{self._below_lidar}m起向上)"
            if self._crop_base
            else f"map z∈[{self._cloud_z_min},{self._cloud_z_max}]"
        )
        odom_desc = (
            f"{in_odom}+TF -> {prefix}state_estimation"
            if self._publish_odom
            else "odom 由 tf_manager 发布 /state_estimation"
        )
        self.get_logger().info(
            f"ego_go2_bridge: {odom_desc}; {in_cloud} (+fallback {fallback_cloud}) "
            f"-> {prefix}cloud_registered_map; 障碍点云 {crop_desc}"
        )

    def _lookup_map_to_base(self) -> TransformException | object:
        timeout = rclpy.duration.Duration(seconds=self._tf_timeout)
        # 对 EGO 的局部栅格更新来说，“最新位姿”比“严格时间对齐”更重要：
        # map->odom 往往不是连续高频 TF（可能仅在重定位/ICP refine 时更新），用 stamp 查会大量失败，
        # 导致 /drone_0_state_estimation 只有 1~2Hz，表现为障碍栅格中心钉在起点。
        return self._buffer.lookup_transform(
            self._map_frame,
            self._base_frame,
            Time(),
            timeout=timeout,
        )

    def _odom_in_map(self, msg: Odometry) -> Odometry | None:
        """与 tf_manager 一致：直接查 TF map->base_link，避免手写合成与 map->base 误用。"""
        try:
            tf = self._lookup_map_to_base()
        except TransformException:
            return None

        t = tf.transform.translation
        r = tf.transform.rotation
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._map_frame
        out.child_frame_id = self._base_frame
        out.pose.pose.position.x = float(t.x)
        out.pose.pose.position.y = float(t.y)
        out.pose.pose.position.z = float(t.z)
        out.pose.pose.orientation = r
        out.twist = msg.twist
        return out

    def _on_odom(self, msg: Odometry) -> None:
        if self._odom_pub is None:
            return
        mapped = self._odom_in_map(msg)
        if mapped is None:
            return
        self._odom_pub.publish(mapped)

    def _check_cloud_freshness(self) -> None:
        if self._last_cloud_rx_sec <= 0.0:
            return
        age = self.get_clock().now().nanoseconds * 1e-9 - self._last_cloud_rx_sec
        if age > float(self.get_parameter("cloud_stale_warn_sec").value):
            self.get_logger().warn(
                f"{self.get_parameter('cloud_input_topic').value} "
                f"已 {age:.2f}s 无新帧，EGO 障碍栅格不会更新",
                throttle_duration_sec=3.0,
            )

    def _cloud_to_map_frame(self, msg: PointCloud2) -> PointCloud2:
        if (msg.header.frame_id or "").strip() == self._map_frame:
            return msg
        stamp = Time.from_msg(msg.header.stamp)
        timeout = rclpy.duration.Duration(seconds=self._tf_timeout)
        tf = None
        # map->odom 常在 ICP refine 时跳变，stamp 对齐失败率高；优先最新 TF 保 EGO 栅格 ~10Hz
        for t in (Time(), stamp if stamp.nanoseconds > 0 else None):
            if t is None:
                continue
            try:
                tf = self._buffer.lookup_transform(
                    self._map_frame,
                    msg.header.frame_id,
                    t,
                    timeout=timeout,
                )
                break
            except TransformException:
                continue
        if tf is None:
            return msg

        t = tf.transform.translation
        q = tf.transform.rotation
        pts = []
        for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pts.append(self._transform_point(float(x), float(y), float(z), t, q))
        if not pts:
            return msg
        header = msg.header
        header.frame_id = self._map_frame
        return point_cloud2.create_cloud_xyz32(header, pts)

    @staticmethod
    def _transform_point(
        x: float, y: float, z: float, t, q
    ) -> list[float]:
        qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        rx = (1.0 - 2.0 * (yy + zz)) * x + 2.0 * (xy - wz) * y + 2.0 * (xz + wy) * z
        ry = 2.0 * (xy + wz) * x + (1.0 - 2.0 * (xx + zz)) * y + 2.0 * (yz - wx) * z
        rz = 2.0 * (xz - wy) * x + 2.0 * (yz + wx) * y + (1.0 - 2.0 * (xx + yy)) * z
        return [rx + float(t.x), ry + float(t.y), rz + float(t.z)]

    @staticmethod
    def _point_map_to_base(
        xm: float, ym: float, zm: float, t, q
    ) -> tuple[float, float, float]:
        """map->base 的逆变换：p_base = R^T (p_map - t)。"""
        qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
        tx, ty, tz = float(t.x), float(t.y), float(t.z)
        dx, dy, dz = xm - tx, ym - ty, zm - tz
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        bx = (1.0 - 2.0 * (yy + zz)) * dx + 2.0 * (xy + wz) * dy + 2.0 * (xz - wy) * dz
        by = 2.0 * (xy - wz) * dx + (1.0 - 2.0 * (xx + zz)) * dy + 2.0 * (yz + wx) * dz
        bz = 2.0 * (xz + wy) * dx + 2.0 * (yz - wx) * dy + (1.0 - 2.0 * (xx + yy)) * dz
        return bx, by, bz

    def _crop_cloud_z(self, msg: PointCloud2) -> PointCloud2:
        if not self._crop_cloud:
            return msg
        kept: list[tuple[float, float, float]] = []
        if self._crop_base:
            timeout = rclpy.duration.Duration(seconds=self._tf_timeout)
            tf = None
            stamp = Time.from_msg(msg.header.stamp)
            for t in (Time(), stamp if stamp.nanoseconds > 0 else None):
                if t is None:
                    continue
                try:
                    tf = self._buffer.lookup_transform(
                        self._map_frame,
                        self._base_frame,
                        t,
                        timeout=timeout,
                    )
                    break
                except TransformException:
                    continue
            if tf is None:
                return msg
            t = tf.transform.translation
            q = tf.transform.rotation
            for x, y, z in point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            ):
                _, _, zb = self._point_map_to_base(float(x), float(y), float(z), t, q)
                if self._z_min_base <= zb <= self._max_h_base:
                    kept.append((float(x), float(y), float(z)))
        else:
            kept = [
                (x, y, z)
                for x, y, z in point_cloud2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True
                )
                if self._cloud_z_min <= float(z) <= self._cloud_z_max
            ]
        if not kept:
            return msg
        header = msg.header
        header.frame_id = self._map_frame
        return point_cloud2.create_cloud_xyz32(header, kept)

    def _publish_cloud(self, msg: PointCloud2, source: str) -> None:
        if (msg.header.frame_id or "").strip() == self._map_frame:
            mapped = msg
        else:
            mapped = self._cloud_to_map_frame(msg)
        cropped = self._crop_cloud_z(mapped)
        cropped.header.stamp = msg.header.stamp
        self._last_cloud_rx_sec = self.get_clock().now().nanoseconds * 1e-9
        self._cloud_pub.publish(cropped)

    def _on_cloud_primary(self, msg: PointCloud2) -> None:
        self._last_primary_cloud_rx_sec = self.get_clock().now().nanoseconds * 1e-9
        self._publish_cloud(msg, "primary")

    def _on_cloud_fallback(self, msg: PointCloud2) -> None:
        if self._last_primary_cloud_rx_sec <= 0.0:
            self._publish_cloud(msg, "fallback")
            return
        age = self.get_clock().now().nanoseconds * 1e-9 - self._last_primary_cloud_rx_sec
        if age >= self._cloud_fallback_stale:
            self._publish_cloud(msg, "fallback")


def main() -> None:
    rclpy.init()
    node = EgoGo2BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
