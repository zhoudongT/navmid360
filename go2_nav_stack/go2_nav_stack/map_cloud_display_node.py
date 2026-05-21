#!/usr/bin/env python3
"""将 /cloud_registered 变换到 map 系发布，与先验地图/ICP 白点云对齐显示。

重定位完成前不发布（避免 map=odom 占位时点云在 RViz 里「旋转漂移」）。
重定位后使用锁定的 T_map_odom + 实时 T_odom_livox，与 tf_manager 一致且 map->odom 不再被改。
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String
import tf2_ros
from tf2_ros import TransformException


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _stamp_to_mat4(stamp) -> np.ndarray:
    t = stamp.transform.translation
    r = stamp.transform.rotation
    rot = _quat_to_rot(r.x, r.y, r.z, r.w)
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rot
    mat[0, 3] = t.x
    mat[1, 3] = t.y
    mat[2, 3] = t.z
    return mat


class MapCloudDisplayNode(Node):
    def __init__(self) -> None:
        super().__init__("map_cloud_display_node")
        self.declare_parameter("input_topic", "/cloud_registered")
        self.declare_parameter("output_topic", "/cloud_registered_map")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("tf_timeout_sec", 0.25)
        self.declare_parameter("require_reloc_ok", True)
        self.declare_parameter("max_points", 12000)
        self.declare_parameter("min_publish_interval_sec", 0.08)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self._require_reloc = bool(self.get_parameter("require_reloc_ok").value)
        self._max_points = max(1000, int(self.get_parameter("max_points").value))
        self._min_interval = float(self.get_parameter("min_publish_interval_sec").value)
        self._last_pub_time = 0.0
        self._reloc_ok = False
        self._t_map_odom: np.ndarray | None = None

        cache = rclpy.duration.Duration(seconds=30.0)
        self._tf_buffer = tf2_ros.Buffer(cache_time=cache)
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(PointCloud2, out_topic, 2)
        self.create_subscription(
            PointCloud2, in_topic, self._on_cloud, qos_profile_sensor_data
        )
        self.create_subscription(Bool, "/relocalization/ok", self._on_reloc_ok, 10)
        self.create_subscription(String, "/relocalization/status", self._on_reloc_status, 10)
        self.get_logger().info(
            f"{in_topic} -> {out_topic} (frame={self._target_frame}, "
            f"require_reloc_ok={self._require_reloc})"
        )

    def _on_reloc_ok(self, msg: Bool) -> None:
        if msg.data and not self._reloc_ok:
            self._refresh_map_odom_lock()
        elif not msg.data:
            self._reloc_ok = False
            self._t_map_odom = None

    def _on_reloc_status(self, msg: String) -> None:
        if msg.data.strip() == "RELOCALIZED":
            self._reloc_ok = True
            self._refresh_map_odom_lock()

    def _refresh_map_odom_lock(self) -> None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._target_frame,
                self._odom_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except TransformException as ex:
            self.get_logger().warn(f"锁定 map->odom 失败: {ex}")
            return
        self._t_map_odom = _stamp_to_mat4(tf)
        self._reloc_ok = True
        t = tf.transform.translation
        self.get_logger().info(
            f"map 已锁定: T_map_odom=({t.x:.2f},{t.y:.2f},{t.z:.2f})，"
            f"{self.get_parameter('output_topic').value} 将与先验图对齐"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        if not msg.header.frame_id:
            return
        if self._require_reloc and not self._reloc_ok:
            self.get_logger().warn(
                "未重定位，跳过 /cloud_registered_map（请 Qt「当前姿态」+ ICP 完成后再显示）",
                throttle_duration_sec=8.0,
            )
            return
        if self._t_map_odom is None:
            self._refresh_map_odom_lock()
            if self._t_map_odom is None:
                return

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_pub_time < self._min_interval:
            return

        try:
            t_odom_src = self._tf_buffer.lookup_transform(
                self._odom_frame,
                msg.header.frame_id,
                Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except TransformException:
            self.get_logger().warn(
                f"TF {msg.header.frame_id}->{self._odom_frame} 不可用",
                throttle_duration_sec=5.0,
            )
            return

        t_odom_src_mat = _stamp_to_mat4(t_odom_src)
        t_map_src = self._t_map_odom @ t_odom_src_mat
        rot = t_map_src[:3, :3]
        trans = t_map_src[:3, 3]

        raw = list(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        )
        if not raw:
            return
        step = max(1, len(raw) // self._max_points)
        pts = []
        for i in range(0, len(raw), step):
            p = raw[i]
            v = np.array([p[0], p[1], p[2]], dtype=np.float64)
            w = rot @ v + trans
            pts.append((float(w[0]), float(w[1]), float(w[2])))

        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self._target_frame
        self._pub.publish(point_cloud2.create_cloud_xyz32(hdr, pts))
        self._last_pub_time = now


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
