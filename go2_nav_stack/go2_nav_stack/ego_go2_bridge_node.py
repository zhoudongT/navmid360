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
        self.declare_parameter("prefer_cloud_stamp_tf", True)
        self.declare_parameter("cloud_tf_fallback_latest", True)
        self.declare_parameter("crop_tf_fallback_latest", True)
        self.declare_parameter("publish_grid_odom", True)
        self.declare_parameter("cloud_stale_warn_sec", 0.35)
        # 相对 base_link：激光下 30cm 至向上高度带内的点作为 EGO 障碍（可通行区外）
        self.declare_parameter("cloud_crop_in_base_frame", True)
        self.declare_parameter("lidar_height_m", 0.27)
        self.declare_parameter("obstacle_below_lidar_m", 0.30)
        self.declare_parameter("obstacle_max_height_above_base_m", 1.0)
        self.declare_parameter("enable_cloud_z_crop", True)
        # map 系绝对高度裁剪（仅 cloud_crop_in_base_frame=false 时使用）
        self.declare_parameter("cloud_z_min_m", 0.05)
        self.declare_parameter("cloud_z_max_m", 0.48)
        self.declare_parameter("robot_body_radius_m", 0.35)

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
        self._robot_body_r = float(self.get_parameter("robot_body_radius_m").value)
        prefix = f"/drone_{did}_"
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self._prefer_cloud_stamp = bool(
            self.get_parameter("prefer_cloud_stamp_tf").value
        )
        self._tf_fallback_latest = bool(
            self.get_parameter("cloud_tf_fallback_latest").value
        )
        self._crop_tf_fallback_latest = bool(
            self.get_parameter("crop_tf_fallback_latest").value
        )
        self._publish_grid_odom = bool(self.get_parameter("publish_grid_odom").value)

        self._publish_odom = bool(self.get_parameter("publish_state_estimation").value)
        self._odom_pub = None
        if self._publish_odom:
            self._odom_pub = self.create_publisher(
                Odometry, f"{prefix}state_estimation", 20
            )

        self._cloud_pub = self.create_publisher(
            PointCloud2, f"{prefix}cloud_registered_map", 10
        )
        self._grid_odom_pub = None
        if self._publish_grid_odom:
            self._grid_odom_pub = self.create_publisher(
                Odometry, f"{prefix}grid_odom", 10
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
        self._last_publish_sec = 0.0
        self._last_publish_source = ""
        self._publish_count = 0
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
        grid_desc = (
            f"{prefix}grid_odom(与点云同 stamp)"
            if self._publish_grid_odom
            else "grid_odom 关闭"
        )
        tf_desc = (
            "点云/栅格 TF=消息 stamp"
            if self._prefer_cloud_stamp
            else "点云/栅格 TF=最新"
        )
        self.get_logger().info(
            f"ego_go2_bridge: {odom_desc}; {in_cloud} (+fallback {fallback_cloud}) "
            f"-> {prefix}cloud_registered_map; {grid_desc}; {tf_desc}; 障碍点云 {crop_desc}"
        )

    def _lookup_transform(
        self, target_frame: str, source_frame: str, stamp: Time
    ) -> object | None:
        """优先点云时间戳对齐 TF，避免障碍栅格与机体位姿错位。"""
        timeout = rclpy.duration.Duration(seconds=self._tf_timeout)
        times: list[Time] = []
        if self._prefer_cloud_stamp and stamp.nanoseconds > 0:
            times.append(stamp)
        if self._tf_fallback_latest or not times:
            times.append(Time())
        last_exc: TransformException | None = None
        for t in times:
            try:
                return self._buffer.lookup_transform(
                    target_frame, source_frame, t, timeout=timeout
                )
            except TransformException as exc:
                last_exc = exc
        if last_exc is not None:
            self.get_logger().debug(
                f"TF {source_frame}->{target_frame} 失败: {last_exc}"
            )
        return None

    def _lookup_map_to_base(
        self, stamp: Time | None = None, *, allow_latest_fallback: bool = True
    ) -> object | None:
        t = stamp if stamp is not None else Time()
        tf = self._lookup_transform(self._map_frame, self._base_frame, t)
        if tf is not None or not allow_latest_fallback:
            return tf
        if stamp is not None and stamp.nanoseconds > 0:
            return self._lookup_transform(self._map_frame, self._base_frame, Time())
        return None

    @staticmethod
    def _odom_from_map_tf(
        tf, stamp, map_frame: str, base_frame: str, twist=None
    ) -> Odometry:
        t = tf.transform.translation
        r = tf.transform.rotation
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = map_frame
        out.child_frame_id = base_frame
        out.pose.pose.position.x = float(t.x)
        out.pose.pose.position.y = float(t.y)
        out.pose.pose.position.z = float(t.z)
        out.pose.pose.orientation = r
        if twist is not None:
            out.twist = twist
        return out

    def _odom_in_map(self, msg: Odometry) -> Odometry | None:
        """与 tf_manager 一致：map->base；用于可选的 state_estimation 转发。"""
        stamp = Time.from_msg(msg.header.stamp)
        tf = self._lookup_map_to_base(stamp)
        if tf is None:
            return None
        return self._odom_from_map_tf(
            tf, msg.header.stamp, self._map_frame, self._base_frame, msg.twist
        )

    def _on_odom(self, msg: Odometry) -> None:
        if self._odom_pub is None:
            return
        mapped = self._odom_in_map(msg)
        if mapped is None:
            return
        self._odom_pub.publish(mapped)

    def _check_cloud_freshness(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        warn_sec = float(self.get_parameter("cloud_stale_warn_sec").value)
        if self._last_publish_sec > 0.0:
            pub_age = now - self._last_publish_sec
            if pub_age > warn_sec:
                self.get_logger().warn(
                    f"EGO 障碍点云已 {pub_age:.1f}s 未发布"
                    f"（最近来源={self._last_publish_source or '无'}），"
                    f"主话题 {self.get_parameter('cloud_input_topic').value} "
                    f"fallback {self.get_parameter('cloud_fallback_topic').value}",
                    throttle_duration_sec=3.0,
                )
            return
        if self._last_primary_cloud_rx_sec > 0.0:
            age = now - self._last_primary_cloud_rx_sec
            if age > warn_sec:
                self.get_logger().warn(
                    f"{self.get_parameter('cloud_input_topic').value} "
                    f"已 {age:.1f}s 无新帧，等待 fallback",
                    throttle_duration_sec=3.0,
                )

    def _cloud_to_map_frame(self, msg: PointCloud2) -> PointCloud2 | None:
        if (msg.header.frame_id or "").strip() == self._map_frame:
            return msg
        stamp = Time.from_msg(msg.header.stamp)
        src = (msg.header.frame_id or "").strip()
        if not src:
            return None
        tf = self._lookup_transform(self._map_frame, src, stamp)
        if tf is None:
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        pts = []
        for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pts.append(self._transform_point(float(x), float(y), float(z), t, q))
        if not pts:
            return None
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

    def _crop_cloud_z(self, msg: PointCloud2) -> PointCloud2 | None:
        if not self._crop_cloud:
            return msg
        kept: list[tuple[float, float, float]] = []
        if self._crop_base:
            stamp = Time.from_msg(msg.header.stamp)
            tf = self._lookup_map_to_base(
                stamp, allow_latest_fallback=self._crop_tf_fallback_latest
            )
            if tf is None:
                return None
            t = tf.transform.translation
            q = tf.transform.rotation
            for x, y, z in point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            ):
                bx, by, zb = self._point_map_to_base(float(x), float(y), float(z), t, q)
                if math.hypot(bx, by) < self._robot_body_r:
                    continue
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
        if mapped is None:
            self.get_logger().warn(
                f"点云({source}) TF 对齐失败，跳过本帧（避免障碍与机体错位）",
                throttle_duration_sec=2.0,
            )
            return
        cropped = self._crop_cloud_z(mapped)
        if cropped is None:
            return
        cropped.header.stamp = msg.header.stamp

        if self._grid_odom_pub is not None:
            stamp = Time.from_msg(msg.header.stamp)
            tf_base = self._lookup_map_to_base(
                stamp, allow_latest_fallback=self._crop_tf_fallback_latest
            )
            if tf_base is not None:
                self._grid_odom_pub.publish(
                    self._odom_from_map_tf(
                        tf_base,
                        msg.header.stamp,
                        self._map_frame,
                        self._base_frame,
                    )
                )

        now = self.get_clock().now().nanoseconds * 1e-9
        self._last_cloud_rx_sec = now
        self._last_publish_sec = now
        self._last_publish_source = source
        self._cloud_pub.publish(cropped)
        self._publish_count += 1
        if self._publish_count == 1 or self._publish_count % 20 == 0:
            self.get_logger().info(
                f"EGO 障碍点云 #{self._publish_count} src={source} "
                f"frame={cropped.header.frame_id} "
                f"stamp={cropped.header.stamp.sec}.{cropped.header.stamp.nanosec:09d}",
                throttle_duration_sec=5.0,
            )

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
