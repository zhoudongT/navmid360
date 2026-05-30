#!/usr/bin/env python3
"""将 FAST-LIO /cloud_registered（含 intensity）变换到 map 系。

- 用手动 4x4 变换保留 intensity（tf2 do_transform_cloud 常会丢通道）。
- /transformed_cloud 由 icp_node 在锁定后用实时 TF 发布，勿与此处混用。
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String
import tf2_ros
from tf2_ros import TransformException


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    r0 = (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy))
    r1 = (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx))
    r2 = (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy))
    return r0, r1, r2


class MapCloudDisplayNode(Node):
    def __init__(self) -> None:
        super().__init__("map_cloud_display_node")
        self.declare_parameter("input_topic", "/cloud_registered")
        self.declare_parameter("output_topic", "/cloud_registered_map")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("tf_timeout_sec", 0.25)
        self.declare_parameter("require_reloc_ok", True)
        self.declare_parameter("use_latest_tf", True)
        self.declare_parameter("max_points", 12000)
        self.declare_parameter("min_publish_interval_sec", 0.08)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("enable_base_z_crop", True)
        # 地面在 base 下方 0.3m；障碍检测带 [ground-margin, max]
        self.declare_parameter("ground_below_base_m", 0.30)
        self.declare_parameter("obstacle_z_margin_below_ground_m", 0.08)
        self.declare_parameter("obstacle_z_max_in_base_m", 0.8)
        self.declare_parameter("robot_body_radius_m", 0.35)

        self._base_frame = str(self.get_parameter("base_frame").value)
        self._crop_base_z = bool(self.get_parameter("enable_base_z_crop").value)
        ground = float(self.get_parameter("ground_below_base_m").value)
        margin = float(self.get_parameter("obstacle_z_margin_below_ground_m").value)
        self._z_min_base = -ground - margin
        self._z_max_base = float(self.get_parameter("obstacle_z_max_in_base_m").value)
        self._robot_body_r = float(self.get_parameter("robot_body_radius_m").value)
        self._in_topic = str(self.get_parameter("input_topic").value)
        self._out_topic = str(self.get_parameter("output_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self._require_reloc = bool(self.get_parameter("require_reloc_ok").value)
        self._use_latest_tf = bool(self.get_parameter("use_latest_tf").value)
        self._max_points = max(1000, int(self.get_parameter("max_points").value))
        self._min_interval = float(self.get_parameter("min_publish_interval_sec").value)
        self._last_pub_time = 0.0
        self._reloc_ok = False
        self._warned_no_intensity = False

        cache = rclpy.duration.Duration(seconds=30.0)
        self._tf_buffer = tf2_ros.Buffer(cache_time=cache)
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(PointCloud2, self._out_topic, 2)

        self.create_subscription(
            PointCloud2, self._in_topic, self._on_lio_cloud, qos_profile_sensor_data
        )
        self.create_subscription(Bool, "/relocalization/ok", self._on_reloc_ok, 10)
        self.create_subscription(String, "/relocalization/status", self._on_reloc_status, 10)
        self.get_logger().info(
            f"/cloud_registered_map: {self._in_topic} -> {self._out_topic} "
            f"(TF->{self._target_frame}, base z=[{self._z_min_base:.2f},{self._z_max_base:.2f}]m)"
            if self._crop_base_z
            else f"/cloud_registered_map: {self._in_topic} -> {self._out_topic} (TF->{self._target_frame})"
        )

    def _set_reloc_ok(self, ok: bool) -> None:
        self._reloc_ok = ok
        if ok:
            self.get_logger().info("重定位完成：发布彩色 /cloud_registered_map")

    def _on_reloc_ok(self, msg: Bool) -> None:
        if msg.data:
            self._set_reloc_ok(True)

    def _on_reloc_status(self, msg: String) -> None:
        if msg.data.strip() == "RELOCALIZED":
            self._set_reloc_ok(True)

    def _gate_reloc(self) -> bool:
        if self._require_reloc and not self._reloc_ok:
            self.get_logger().warn(
                "未重定位，跳过 /cloud_registered_map",
                throttle_duration_sec=8.0,
            )
            return False
        return True

    def _throttle_ok(self) -> bool:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_pub_time < self._min_interval:
            return False
        self._last_pub_time = now
        return True

    def _intensity_field(self, msg: PointCloud2) -> str | None:
        for f in msg.fields:
            if f.name in ("intensity", "reflectivity", "i"):
                return f.name
        return None

    def _lookup_map_transform(self, src_frame: str, stamp: Time):
        timeout = rclpy.duration.Duration(seconds=self._tf_timeout)
        times = [Time()] if self._use_latest_tf else []
        if stamp.nanoseconds > 0:
            times.insert(0, stamp)
        elif not times:
            times = [Time()]
        last_ex = None
        for t in times:
            try:
                return self._tf_buffer.lookup_transform(
                    self._target_frame,
                    src_frame,
                    t,
                    timeout=timeout,
                )
            except TransformException as ex:
                last_ex = ex
        raise last_ex  # type: ignore[misc]

    def _lookup_map_to_base(self, stamp: Time):
        timeout = rclpy.duration.Duration(seconds=self._tf_timeout)
        times = [Time()] if self._use_latest_tf else []
        if stamp.nanoseconds > 0:
            times.insert(0, stamp)
        elif not times:
            times = [Time()]
        last_ex = None
        for t in times:
            try:
                return self._tf_buffer.lookup_transform(
                    self._target_frame,
                    self._base_frame,
                    t,
                    timeout=timeout,
                )
            except TransformException as ex:
                last_ex = ex
        raise last_ex  # type: ignore[misc]

    @staticmethod
    def _point_map_to_base(
        x: float, y: float, z: float, t, q
    ) -> tuple[float, float, float]:
        tx, ty, tz = float(t.x), float(t.y), float(t.z)
        dx, dy, dz = x - tx, y - ty, z - tz
        qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        bx = (1.0 - 2.0 * (yy + zz)) * dx + 2.0 * (xy + wz) * dy + 2.0 * (xz - wy) * dz
        by = 2.0 * (xy - wz) * dx + (1.0 - 2.0 * (xx + zz)) * dy + 2.0 * (yz + wx) * dz
        bz = 2.0 * (xz + wy) * dx + 2.0 * (yz - wx) * dy + (1.0 - 2.0 * (xx + yy)) * dz
        return bx, by, bz

    def _filter_by_base_z(self, msg: PointCloud2, stamp: Time) -> PointCloud2 | None:
        if not self._crop_base_z:
            return msg
        try:
            tf_mb = self._lookup_map_to_base(stamp)
        except TransformException as ex:
            self.get_logger().warn(
                f"map->{self._base_frame} TF 失败，跳过 z 裁剪: {ex}",
                throttle_duration_sec=5.0,
            )
            return None
        t = tf_mb.transform.translation
        q = tf_mb.transform.rotation
        intensity_name = self._intensity_field(msg)
        kept: list[tuple] = []
        if intensity_name:
            for row in point_cloud2.read_points(
                msg, field_names=("x", "y", "z", intensity_name), skip_nans=True
            ):
                bx, by, bz = self._point_map_to_base(
                    float(row[0]), float(row[1]), float(row[2]), t, q
                )
                if math.hypot(bx, by) < self._robot_body_r:
                    continue
                if self._z_min_base <= bz <= self._z_max_base:
                    kept.append((float(row[0]), float(row[1]), float(row[2]), float(row[3])))
            if not kept:
                return msg
            hdr = Header()
            hdr.stamp = msg.header.stamp
            hdr.frame_id = msg.header.frame_id
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(
                    name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
                ),
            ]
            return point_cloud2.create_cloud(hdr, fields, kept)
        for x, y, z in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            bx, by, bz = self._point_map_to_base(float(x), float(y), float(z), t, q)
            if math.hypot(bx, by) < self._robot_body_r:
                continue
            if self._z_min_base <= bz <= self._z_max_base:
                kept.append((float(x), float(y), float(z)))
        if not kept:
            return msg
        hdr = Header()
        hdr.stamp = msg.header.stamp
        hdr.frame_id = msg.header.frame_id
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        return point_cloud2.create_cloud(hdr, fields, kept)

    def _transform_to_map(self, msg: PointCloud2, trans) -> PointCloud2:
        t = trans.transform.translation
        q = trans.transform.rotation
        r0, r1, r2 = _quat_to_rot(q.x, q.y, q.z, q.w)
        tx, ty, tz = float(t.x), float(t.y), float(t.z)

        def map_pt(x: float, y: float, z: float) -> tuple[float, float, float]:
            nx = r0[0] * x + r0[1] * y + r0[2] * z + tx
            ny = r1[0] * x + r1[1] * y + r1[2] * z + ty
            nz = r2[0] * x + r2[1] * y + r2[2] * z + tz
            return nx, ny, nz

        intensity_name = self._intensity_field(msg)
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self._target_frame

        if intensity_name:
            raw = list(
                point_cloud2.read_points(
                    msg,
                    field_names=("x", "y", "z", intensity_name),
                    skip_nans=True,
                )
            )
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(
                    name="intensity",
                    offset=12,
                    datatype=PointField.FLOAT32,
                    count=1,
                ),
            ]
            pts = []
            for row in raw:
                nx, ny, nz = map_pt(float(row[0]), float(row[1]), float(row[2]))
                pts.append((nx, ny, nz, float(row[3])))
            return point_cloud2.create_cloud(hdr, fields, pts)

        if not self._warned_no_intensity:
            self._warned_no_intensity = True
            names = [f.name for f in msg.fields]
            self.get_logger().warn(
                f"/cloud_registered 无 intensity 字段 (fields={names})，仅发布 xyz"
            )
        raw = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pts = [map_pt(float(r[0]), float(r[1]), float(r[2])) for r in raw]
        return point_cloud2.create_cloud(hdr, fields, pts)

    def _downsample_cloud(self, msg: PointCloud2) -> PointCloud2:
        intensity_name = self._intensity_field(msg)
        if intensity_name:
            field_names = ("x", "y", "z", "intensity")
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(
                    name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
                ),
            ]
        else:
            field_names = ("x", "y", "z")
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]

        raw = list(point_cloud2.read_points(msg, field_names=field_names, skip_nans=True))
        if len(raw) <= self._max_points:
            return msg
        step = max(1, len(raw) // self._max_points)
        pts = [raw[i] for i in range(0, len(raw), step)]
        hdr = Header()
        hdr.stamp = msg.header.stamp
        hdr.frame_id = msg.header.frame_id
        return point_cloud2.create_cloud(hdr, fields, pts)

    def _on_lio_cloud(self, msg: PointCloud2) -> None:
        if not self._gate_reloc() or not msg.header.frame_id or not self._throttle_ok():
            return
        try:
            stamp = Time.from_msg(msg.header.stamp)
            trans = self._lookup_map_transform(msg.header.frame_id, stamp)
            out = self._transform_to_map(msg, trans)
            filtered = self._filter_by_base_z(out, stamp)
            if filtered is None:
                return
            self._pub.publish(self._downsample_cloud(filtered))
        except TransformException as ex:
            self.get_logger().warn(
                f"TF {msg.header.frame_id}->{self._target_frame}: {ex}",
                throttle_duration_sec=5.0,
            )


def main() -> None:
    rclpy.init()
    node = MapCloudDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
