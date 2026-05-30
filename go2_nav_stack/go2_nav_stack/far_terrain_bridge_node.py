#!/usr/bin/env python3
"""将 Go2 点云转为 FAR Planner 所需的 terrain_map / registered_scan 话题。

FAR 通过点云 intensity 区分可通行/障碍（intensity < terrain_free_Z 为 free）。
本节点用机体系高度带做简化编码，适用于室内平面导航（is_static_env=true）。
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros
from tf2_ros import TransformException


class FarTerrainBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("far_terrain_bridge")
        self.declare_parameter("cloud_input_topic", "/cloud_registered_map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 0.35)
        self.declare_parameter("terrain_range_m", 15.0)
        self.declare_parameter("scan_range_m", 5.0)
        self.declare_parameter("voxel_m", 0.12)
        self.declare_parameter("obstacle_z_min_m", 0.17)
        self.declare_parameter("obstacle_z_max_m", 1.0)
        self.declare_parameter("ground_z_max_m", 0.17)
        self.declare_parameter("obstacle_intensity", 2.0)
        self.declare_parameter("free_intensity", 0.05)
        self.declare_parameter("robot_body_radius_m", 0.52)
        # 障碍点在 map 平面额外膨胀（米），与 FAR obs_inflate_size 叠加
        self.declare_parameter("obstacle_xy_inflate_m", 0.15)
        self.declare_parameter("require_reloc_ok", True)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self._terrain_r = float(self.get_parameter("terrain_range_m").value)
        self._scan_r = float(self.get_parameter("scan_range_m").value)
        self._voxel = float(self.get_parameter("voxel_m").value)
        self._z_obs_lo = float(self.get_parameter("obstacle_z_min_m").value)
        self._z_obs_hi = float(self.get_parameter("obstacle_z_max_m").value)
        self._z_ground_hi = float(self.get_parameter("ground_z_max_m").value)
        self._i_obs = float(self.get_parameter("obstacle_intensity").value)
        self._i_free = float(self.get_parameter("free_intensity").value)
        self._body_r = float(self.get_parameter("robot_body_radius_m").value)
        self._obs_inflate_m = max(
            0.0, float(self.get_parameter("obstacle_xy_inflate_m").value)
        )
        self._inflate_steps = max(
            0, int(math.ceil(self._obs_inflate_m / max(self._voxel, 1e-3)))
        )
        self._require_reloc = bool(self.get_parameter("require_reloc_ok").value)
        self._reloc_ok = not self._require_reloc

        self._terrain_pub = self.create_publisher(PointCloud2, "/terrain_map", 5)
        self._terrain_ext_pub = self.create_publisher(PointCloud2, "/terrain_map_ext", 5)
        self._scan_pub = self.create_publisher(PointCloud2, "/registered_scan", 5)

        self._buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._listener = tf2_ros.TransformListener(self._buffer, self)

        topic = str(self.get_parameter("cloud_input_topic").value)
        self.create_subscription(PointCloud2, topic, self._on_cloud, 10)
        if self._require_reloc:
            self.create_subscription(Bool, "/relocalization/ok", self._on_reloc_ok, 10)
        self._count = 0
        self.get_logger().info(
            f"far_terrain_bridge: {topic} -> /terrain_map, /terrain_map_ext, /registered_scan "
            f"(require_reloc_ok={self._require_reloc})"
        )

    def _on_reloc_ok(self, msg: Bool) -> None:
        if msg.data:
            self._reloc_ok = True
            self.get_logger().info("重定位 OK，开始向 FAR 发布 terrain/scan")

    def _lookup_base_tf(self, stamp: Time):
        try:
            return self._buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                stamp,
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except TransformException:
            try:
                return self._buffer.lookup_transform(
                    self._map_frame,
                    self._base_frame,
                    Time(),
                    timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
                )
            except TransformException as exc:
                self.get_logger().warn(
                    f"TF {self._map_frame}->{self._base_frame} 失败: {exc}",
                    throttle_duration_sec=3.0,
                )
                return None

    @staticmethod
    def _map_to_base(
        xm: float, ym: float, zm: float, t, q
    ) -> tuple[float, float, float]:
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

    def _voxel_key(self, x: float, y: float, z: float) -> tuple[int, int, int]:
        v = self._voxel
        return (int(math.floor(x / v)), int(math.floor(y / v)), int(math.floor(z / v)))

    def _encode_cloud(
        self, msg: PointCloud2, tf, range_xy: float
    ) -> tuple[list[tuple[float, float, float, float]], list[tuple[float, float, float, float]]]:
        t = tf.transform.translation
        q = tf.transform.rotation
        rx = float(t.x)
        ry = float(t.y)
        obs_vox: dict[tuple[int, int, int], tuple[float, float, float, float]] = {}
        free_vox: dict[tuple[int, int, int], tuple[float, float, float, float]] = {}
        r2 = range_xy * range_xy

        for x, y, z in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            xm, ym, zm = float(x), float(y), float(z)
            if (xm - rx) ** 2 + (ym - ry) ** 2 > r2:
                continue
            bx, by, bz = self._map_to_base(xm, ym, zm, t, q)
            if math.hypot(bx, by) < self._body_r:
                continue
            if self._z_obs_lo <= bz <= self._z_obs_hi:
                vx, vy, vz = self._voxel_key(xm, ym, zm)
                obs_vox[(vx, vy, vz)] = (xm, ym, zm, self._i_obs)
                v = self._voxel
                for dx in range(-self._inflate_steps, self._inflate_steps + 1):
                    for dy in range(-self._inflate_steps, self._inflate_steps + 1):
                        if dx == 0 and dy == 0:
                            continue
                        key = (vx + dx, vy + dy, vz)
                        if key not in obs_vox:
                            obs_vox[key] = (
                                xm + dx * v,
                                ym + dy * v,
                                zm,
                                self._i_obs,
                            )
            elif bz <= self._z_ground_hi:
                pt = (xm, ym, zm, self._i_free)
                free_vox[self._voxel_key(xm, ym, zm)] = pt

        terrain_pts = list(obs_vox.values()) + list(free_vox.values())
        scan_pts = list(obs_vox.values())
        return terrain_pts, scan_pts

    @staticmethod
    def _to_msg(header, pts: list[tuple[float, float, float, float]]) -> PointCloud2:
        cloud = [
            (x, y, z, i) for x, y, z, i in pts
        ]
        out = point_cloud2.create_cloud(
            header,
            [
                point_cloud2.PointField(
                    name="x", offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1
                ),
                point_cloud2.PointField(
                    name="y", offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1
                ),
                point_cloud2.PointField(
                    name="z", offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1
                ),
                point_cloud2.PointField(
                    name="intensity",
                    offset=12,
                    datatype=point_cloud2.PointField.FLOAT32,
                    count=1,
                ),
            ],
            cloud,
        )
        return out

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self._require_reloc and not self._reloc_ok:
            return
        stamp = Time.from_msg(msg.header.stamp)
        tf = self._lookup_base_tf(stamp)
        if tf is None:
            return

        terrain_pts, scan_pts = self._encode_cloud(msg, tf, self._terrain_r)
        scan_pts_local, _ = self._encode_cloud(msg, tf, self._scan_r)

        header = msg.header
        header.frame_id = self._map_frame

        if terrain_pts:
            self._terrain_pub.publish(self._to_msg(header, terrain_pts))
            self._terrain_ext_pub.publish(self._to_msg(header, terrain_pts))
        if scan_pts_local:
            self._scan_pub.publish(self._to_msg(header, scan_pts_local))

        self._count += 1
        if self._count == 1 or self._count % 30 == 0:
            self.get_logger().info(
                f"FAR terrain #{self._count} terrain={len(terrain_pts)} scan={len(scan_pts_local)}",
                throttle_duration_sec=5.0,
            )


def main() -> None:
    rclpy.init()
    node = FarTerrainBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
