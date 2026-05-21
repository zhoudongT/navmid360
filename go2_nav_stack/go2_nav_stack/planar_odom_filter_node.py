#!/usr/bin/env python3
"""将 FAST-LIO 6DoF 里程计约束为地面四足平面：z 固定、仅保留 yaw。

含转圈时自适应 yaw 跟随、里程计跳变剔除，供 tf_manager / 滚动 costmap 使用。
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
import tf2
from tf2_ros import Buffer, TransformListener


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


def _pose_to_mat4(pose: Pose) -> np.ndarray:
    o = pose.orientation
    rot = _quat_to_rot(o.x, o.y, o.z, o.w)
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rot
    mat[0, 3] = pose.position.x
    mat[1, 3] = pose.position.y
    mat[2, 3] = pose.position.z
    return mat


def _mat4_to_pose(mat: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x = float(mat[0, 3])
    pose.position.y = float(mat[1, 3])
    pose.position.z = float(mat[2, 3])
    rot = mat[:3, :3]
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    h = yaw * 0.5
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(h)
    pose.orientation.w = math.cos(h)
    return pose


class PlanarOdomFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("planar_odom_filter_node")
        self.declare_parameter("input_topic", "/lio_odom_raw")
        self.declare_parameter("output_topic", "/state_estimation")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("lock_z", True)
        self.declare_parameter("z_fixed", 0.0)
        self.declare_parameter("planar_orientation", True)
        self.declare_parameter("yaw_smooth_alpha", 0.35)
        self.declare_parameter("yaw_smooth_alpha_spin", 0.85)
        self.declare_parameter("spin_angular_vel_threshold", 0.45)
        self.declare_parameter("max_xy_jump_m", 0.0)
        self.declare_parameter("max_yaw_jump_deg", 0.0)
        self.declare_parameter("apply_sensor_to_base_extrinsic", True)
        self.declare_parameter("tf_lookup_timeout", 0.05)
        self.declare_parameter("use_lio_stamp", True)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._lock_z = bool(self.get_parameter("lock_z").value)
        self._z_fixed = float(self.get_parameter("z_fixed").value)
        self._planar = bool(self.get_parameter("planar_orientation").value)
        self._yaw_alpha = float(self.get_parameter("yaw_smooth_alpha").value)
        self._yaw_alpha_spin = float(self.get_parameter("yaw_smooth_alpha_spin").value)
        self._spin_wz_th = float(self.get_parameter("spin_angular_vel_threshold").value)
        self._max_xy_jump = float(self.get_parameter("max_xy_jump_m").value)
        self._max_yaw_jump = math.radians(
            float(self.get_parameter("max_yaw_jump_deg").value)
        )
        self._apply_extrinsic = bool(
            self.get_parameter("apply_sensor_to_base_extrinsic").value
        )
        self._tf_timeout = float(self.get_parameter("tf_lookup_timeout").value)
        self._use_lio_stamp = bool(self.get_parameter("use_lio_stamp").value)
        self._yaw_filt: float | None = None
        self._last_xy: tuple[float, float] | None = None
        self._last_pub_yaw: float | None = None
        self._tf_warned = False
        self._jump_warned = False
        self._lio_count = 0
        self._pub_count = 0
        self._xy_jump_guard = self._max_xy_jump > 0.0
        self._yaw_jump_guard = self._max_yaw_jump > 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(Odometry, out_topic, 20)
        self.create_subscription(Odometry, in_topic, self._on_odom, 20)
        self.create_subscription(Bool, "/relocalization/ok", self._on_reloc_ok, 10)
        self.create_timer(8.0, self._watchdog)
        self.get_logger().info(
            f"{in_topic} -> {out_topic} (planar, xy_jump="
            f"{'off' if not self._xy_jump_guard else f'{self._max_xy_jump}m'}, "
            f"yaw_jump={'off' if not self._yaw_jump_guard else f'{math.degrees(self._max_yaw_jump):.0f}deg'})"
        )

    def _reset_state(self, reason: str) -> None:
        self._yaw_filt = None
        self._last_xy = None
        self._jump_warned = False
        self.get_logger().info(f"里程计滤波状态已重置: {reason}")

    def _on_reloc_ok(self, msg: Bool) -> None:
        if msg.data:
            self._reset_state("重定位完成")

    def _watchdog(self) -> None:
        if self._lio_count == 0:
            self.get_logger().warn(
                f"未收到 {self.get_parameter('input_topic').value}，"
                "请检查 FAST-LIO /lio_odom_raw",
                throttle_duration_sec=15.0,
            )
            return
        if self._pub_count == 0:
            self.get_logger().error(
                f"已收 {self._lio_count} 帧 /lio_odom_raw，但 /state_estimation 从未发布："
                "跳变剔除过严或节点异常，请确认 max_xy_jump_m=0 max_yaw_jump_deg=0",
                throttle_duration_sec=10.0,
            )

    def _pose_odom_to_base(self, msg: Odometry) -> Pose:
        child = (msg.child_frame_id or "imu_link").strip()
        if not self._apply_extrinsic or child == self._base_frame:
            return msg.pose.pose
        dur = rclpy.duration.Duration(seconds=self._tf_timeout)
        last_exc: Exception | None = None
        for stamp in (msg.header.stamp, rclpy.time.Time()):
            try:
                t_base_child = self._tf_buffer.lookup_transform(
                    self._base_frame,
                    child,
                    stamp,
                    timeout=dur,
                )
                t = t_base_child.transform.translation
                r = t_base_child.transform.rotation
                t_bc = np.eye(4, dtype=np.float64)
                t_bc[:3, :3] = _quat_to_rot(r.x, r.y, r.z, r.w)
                t_bc[0, 3] = t.x
                t_bc[1, 3] = t.y
                t_bc[2, 3] = t.z
                t_odom_base = _pose_to_mat4(msg.pose.pose) @ np.linalg.inv(t_bc)
                return _mat4_to_pose(t_odom_base)
            except (tf2.TransformException, Exception) as exc:
                last_exc = exc
        if not self._tf_warned:
            self.get_logger().warn(
                f"odom->{self._base_frame} 外参合成失败 ({child}): {last_exc}；暂用原始位姿"
            )
            self._tf_warned = True
        return msg.pose.pose

    def _reject_jump(self, x: float, y: float, yaw: float) -> bool:
        if self._last_xy is None:
            return False
        if self._xy_jump_guard:
            dx = x - self._last_xy[0]
            dy = y - self._last_xy[1]
            if math.hypot(dx, dy) > self._max_xy_jump:
                return True
        if self._yaw_jump_guard and self._last_pub_yaw is not None:
            if abs(_normalize_angle(yaw - self._last_pub_yaw)) > self._max_yaw_jump:
                return True
        return False

    def _on_odom(self, msg: Odometry) -> None:
        self._lio_count += 1
        base_pose = self._pose_odom_to_base(msg)
        x = float(base_pose.position.x)
        y = float(base_pose.position.y)
        o = base_pose.orientation
        yaw = _yaw_from_quat(o.x, o.y, o.z, o.w)

        if self._reject_jump(x, y, yaw):
            if not self._jump_warned:
                self.get_logger().warn(
                    "丢弃 LIO 里程计跳变（快速转圈/退化时常见），保持上一帧 odom"
                )
                self._jump_warned = True
            return
        self._jump_warned = False

        wz = abs(float(msg.twist.twist.angular.z))
        alpha = (
            self._yaw_alpha_spin
            if wz >= self._spin_wz_th
            else self._yaw_alpha
        )
        if self._planar and alpha > 0.0:
            if self._yaw_filt is None:
                self._yaw_filt = yaw
            else:
                dy = _normalize_angle(yaw - self._yaw_filt)
                self._yaw_filt = _normalize_angle(self._yaw_filt + alpha * dy)
            yaw = self._yaw_filt

        self._last_xy = (x, y)

        out = Odometry()
        out.header.stamp = (
            msg.header.stamp if self._use_lio_stamp else self.get_clock().now().to_msg()
        )
        out.header.frame_id = str(self.get_parameter("odom_frame").value)
        out.child_frame_id = self._base_frame
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = self._z_fixed if self._lock_z else base_pose.position.z
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
        self._pub_count += 1
        self._last_pub_yaw = yaw
        if self._pub_count == 1:
            self.get_logger().info(
                f"首帧 /state_estimation: xy=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.1f}°"
            )


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
