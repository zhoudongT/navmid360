#!/usr/bin/env python3
"""TF 监视与 map->odom 保持；静态外参发布；odom 健康检测。

ICP /icp_result 为 T_map_lidar（当前扫帧 livox -> map），须结合重定位时刻的
T_odom_lidar 合成 T_map_odom = T_map_lidar @ inv(T_odom_lidar)，不能把 ICP 位姿
直接当作 map->odom。
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


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


def _stamp_to_mat4(stamp: TransformStamped) -> np.ndarray:
    t = stamp.transform.translation
    r = stamp.transform.rotation
    rot = _quat_to_rot(r.x, r.y, r.z, r.w)
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rot
    mat[0, 3] = t.x
    mat[1, 3] = t.y
    mat[2, 3] = t.z
    return mat


def _normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _yaw_from_pose_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def _mat4_to_pose(mat: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x = float(mat[0, 3])
    pose.position.y = float(mat[1, 3])
    pose.position.z = float(mat[2, 3])
    rot = mat[:3, :3]
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    qx, qy, qz, qw = _rpy_to_quat(0.0, 0.0, yaw)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


class TfManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("tf_manager_node")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("lidar_frame", "livox_frame")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("tf_timeout", 0.3)
        self.declare_parameter("static_base_to_imu_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("static_base_to_imu_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("static_base_to_lidar_xyz", [0.0, 0.0, 0.15])
        self.declare_parameter("static_base_to_lidar_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("publish_base_to_imu", False)
        self.declare_parameter("publish_odom_to_base", True)
        self.declare_parameter("publish_map_to_odom", True)
        self.declare_parameter("latch_map_to_odom", True)
        self.declare_parameter("allow_icp_refine", True)
        self.declare_parameter("icp_refine_min_interval_sec", 2.0)
        self.declare_parameter("seed_map_to_odom_identity", False)
        self.declare_parameter("seed_odom_to_base_identity", True)
        self.declare_parameter("odom_topic", "/lio_odom_raw")
        self.declare_parameter("republish_state_estimation", True)
        self.declare_parameter("state_estimation_topic", "/state_estimation")
        self.declare_parameter("apply_lio_to_base_extrinsic", False)
        self.declare_parameter("republish_odom_to_base_on_timer", False)
        self.declare_parameter("planar_yaw_only", True)
        self.declare_parameter("lock_odom_z", True)
        self.declare_parameter("odom_z_fixed", 0.0)
        self.declare_parameter("yaw_smooth_alpha", 0.4)
        self.declare_parameter("yaw_smooth_alpha_spin", 0.88)
        self.declare_parameter("spin_angular_vel_threshold", 0.45)
        self.declare_parameter("refresh_map_to_odom_stamp", False)
        self.declare_parameter("spin_block_reloc_wz", 0.55)
        self.declare_parameter("spin_reloc_settle_sec", 2.0)
        self.declare_parameter("icp_refine_max_yaw_deg", 25.0)
        self.declare_parameter("icp_refine_max_xy_m", 0.6)

        self._map_frame = self.get_parameter("map_frame").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._lidar_frame = self.get_parameter("lidar_frame").value
        self._map_to_odom = TransformStamped()
        self._map_to_odom.header.frame_id = self._map_frame
        self._map_to_odom.child_frame_id = self._odom_frame
        self._have_map_odom = False
        self._t_map_odom_mat: np.ndarray | None = None
        self._reloc_applied = False
        self._last_icp_refine_time = 0.0
        self._odom_to_base = TransformStamped()
        self._odom_to_base.header.frame_id = self._odom_frame
        self._odom_to_base.child_frame_id = self._base_frame
        self._have_odom_base = False
        self._odom_from_topic = False

        self._tf_pub = TransformBroadcaster(self)
        self._static_pub = StaticTransformBroadcaster(self)
        self._map_static_pub = StaticTransformBroadcaster(self)
        self._tf_ok_pub = self.create_publisher(Bool, "/navigation/tf_ok", 10)
        self._health_pub = self.create_publisher(String, "/navigation/health", 10)

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)

        self._apply_lio_extrinsic = bool(
            self.get_parameter("apply_lio_to_base_extrinsic").value
        )
        self._planar_yaw = bool(self.get_parameter("planar_yaw_only").value)
        self._lock_z = bool(self.get_parameter("lock_odom_z").value)
        self._z_fixed = float(self.get_parameter("odom_z_fixed").value)
        self._yaw_alpha = float(self.get_parameter("yaw_smooth_alpha").value)
        self._yaw_alpha_spin = float(self.get_parameter("yaw_smooth_alpha_spin").value)
        self._spin_wz_th = float(self.get_parameter("spin_angular_vel_threshold").value)
        self._refresh_map_stamp = bool(
            self.get_parameter("refresh_map_to_odom_stamp").value
        )
        self._yaw_filt: float | None = None
        self._was_spinning = False
        self._last_wz = 0.0
        self._last_odom_mono = 0.0
        self._spin_settle_until = 0.0
        self._spin_block_wz = float(self.get_parameter("spin_block_reloc_wz").value)
        self._spin_settle_sec = float(self.get_parameter("spin_reloc_settle_sec").value)
        self._refine_max_yaw = math.radians(
            float(self.get_parameter("icp_refine_max_yaw_deg").value)
        )
        self._refine_max_xy = float(self.get_parameter("icp_refine_max_xy_m").value)
        self._republish_se = bool(self.get_parameter("republish_state_estimation").value)
        self._republish_odom_timer = bool(
            self.get_parameter("republish_odom_to_base_on_timer").value
        )
        self._se_topic = str(self.get_parameter("state_estimation_topic").value)

        odom_topic = str(self.get_parameter("odom_topic").value)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self._se_pub = None
        if self._republish_se:
            self._se_pub = self.create_publisher(Odometry, self._se_topic, 20)
        self._reloc_ok_pub = self.create_publisher(Bool, "/relocalization/ok", 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/relocalization/pose",
            self._on_reloc_pose,
            10,
        )
        # relocalization_manager 崩溃时仍可用 ICP 结果建立 map->odom
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/icp_result",
            self._on_icp_result_direct,
            10,
        )

        self._publish_static()
        if bool(self.get_parameter("seed_map_to_odom_identity").value):
            self._seed_map_to_odom_identity()
        else:
            self.get_logger().info(
                "map->odom 未发布，等待 ICP 对先验 map 重定位（/relocalization/pose）"
            )
        if bool(self.get_parameter("seed_odom_to_base_identity").value):
            self._seed_odom_to_base_identity()
        period = 1.0 / max(1.0, float(self.get_parameter("publish_rate").value))
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"tf_manager: odom 输入 {odom_topic} -> odom->base；"
            + (
                f" 同步发布 map 系 {self._se_topic}（~lio 频率）"
                if self._republish_se
                else ""
            )
            + f"；map->odom 仅重定位后"
        )

    def _publish_static(self) -> None:
        stamp = self.get_clock().now().to_msg()
        statics: list[TransformStamped] = []

        if bool(self.get_parameter("publish_base_to_imu").value):
            imu = TransformStamped()
            imu.header.stamp = stamp
            imu.header.frame_id = self._base_frame
            imu.child_frame_id = "imu_link"
            ix, iy, iz = self.get_parameter("static_base_to_imu_xyz").value
            ir, ip, iyaw = self.get_parameter("static_base_to_imu_rpy").value
            imu.transform.translation.x = float(ix)
            imu.transform.translation.y = float(iy)
            imu.transform.translation.z = float(iz)
            q = _rpy_to_quat(float(ir), float(ip), float(iyaw))
            imu.transform.rotation.x = q[0]
            imu.transform.rotation.y = q[1]
            imu.transform.rotation.z = q[2]
            imu.transform.rotation.w = q[3]
            statics.append(imu)
            self.get_logger().warn(
                "publish_base_to_imu=true：与 fastlio livox->imu 可能冲突，建议 false"
            )

        lidar = TransformStamped()
        lidar.header.stamp = stamp
        lidar.header.frame_id = self._base_frame
        lidar.child_frame_id = self._lidar_frame
        lx, ly, lz = self.get_parameter("static_base_to_lidar_xyz").value
        lr, lp, lyaw = self.get_parameter("static_base_to_lidar_rpy").value
        lidar.transform.translation.x = float(lx)
        lidar.transform.translation.y = float(ly)
        lidar.transform.translation.z = float(lz)
        q2 = _rpy_to_quat(float(lr), float(lp), float(lyaw))
        lidar.transform.rotation.x = q2[0]
        lidar.transform.rotation.y = q2[1]
        lidar.transform.rotation.z = q2[2]
        lidar.transform.rotation.w = q2[3]
        statics.append(lidar)
        self._static_pub.sendTransform(statics)
        self.get_logger().info(
            f"static TF: {self._base_frame}->{self._lidar_frame} "
            f"(imu via livox->imu from fastlio2 launch)"
        )

    def _apply_map_to_odom_mat(self, t_map_odom: np.ndarray, log_prefix: str) -> None:
        first_lock = not self._reloc_applied
        pose = _mat4_to_pose(t_map_odom)
        self._map_to_odom.header.stamp = self.get_clock().now().to_msg()
        self._map_to_odom.header.frame_id = self._map_frame
        self._map_to_odom.child_frame_id = self._odom_frame
        self._map_to_odom.transform.translation.x = pose.position.x
        self._map_to_odom.transform.translation.y = pose.position.y
        self._map_to_odom.transform.translation.z = pose.position.z
        self._map_to_odom.transform.rotation = pose.orientation
        self._have_map_odom = True
        self._t_map_odom_mat = t_map_odom.copy()
        self._reloc_applied = True
        # map->odom 用静态 TF，避免 50Hz 刷时间戳导致 RViz 里 base_link 抖动
        self._map_static_pub.sendTransform([self._map_to_odom])
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z),
            1.0 - 2.0 * (pose.orientation.z**2),
        )
        self.get_logger().info(
            f"{log_prefix} map->odom: xy=({pose.position.x:.2f},{pose.position.y:.2f}) "
            f"yaw={math.degrees(yaw):.1f}°"
        )
        if first_lock:
            self._reloc_ok_pub.publish(Bool(data=True))

    def _on_icp_result_direct(self, msg: PoseWithCovarianceStamped) -> None:
        if self._reloc_applied:
            return
        self.get_logger().warn(
            "直接收到 /icp_result（relocalization_manager 可能未运行），尝试更新 map->odom"
        )
        self._on_reloc_pose(msg)

    def _seed_odom_to_base_identity(self) -> None:
        """重定位前占位 odom->base，保证 TF 树连通；收到 /state_estimation 后替换。"""
        now = self.get_clock().now().to_msg()
        self._odom_to_base.header.stamp = now
        self._odom_to_base.transform.translation.x = 0.0
        self._odom_to_base.transform.translation.y = 0.0
        self._odom_to_base.transform.translation.z = 0.0
        q = _rpy_to_quat(0.0, 0.0, 0.0)
        self._odom_to_base.transform.rotation.x = q[0]
        self._odom_to_base.transform.rotation.y = q[1]
        self._odom_to_base.transform.rotation.z = q[2]
        self._odom_to_base.transform.rotation.w = q[3]
        self._have_odom_base = True
        self._tf_pub.sendTransform([self._odom_to_base])
        self.get_logger().info(
            f"已发布占位 odom->base_link（单位阵），等待 {self.get_parameter('odom_topic').value} 更新"
        )

    def _seed_map_to_odom_identity(self) -> None:
        """可选占位：map=odom（仅调试）。默认关闭，避免 map/odom 叠在一起。"""
        mat = np.eye(4, dtype=np.float64)
        self._apply_map_to_odom_mat(mat, "Seeded identity (debug)")
        self._reloc_applied = False
        self.get_logger().warn(
            "seed_map_to_odom_identity=true：map 与 odom 重合，先验图不会对齐，仅调试用"
        )

    def _lookup_t_odom_sensor(
        self, timeout_sec: float, stamp: rclpy.time.Time | None = None
    ) -> np.ndarray | None:
        """重定位时刻的 T_odom_sensor，用于与 ICP 的 T_map_sensor 合成 map->odom。"""
        dur = rclpy.duration.Duration(seconds=timeout_sec)
        times: list[rclpy.time.Time] = []
        if stamp is not None and stamp.nanoseconds > 0:
            times.append(stamp)
        times.append(rclpy.time.Time())
        for child in ("livox_frame", "imu_link", self._lidar_frame):
            for t in times:
                try:
                    tf = self._buffer.lookup_transform(
                        self._odom_frame,
                        child,
                        t,
                        timeout=dur,
                    )
                    break
                except Exception:
                    tf = None
            if tf is None:
                continue
            mat = _stamp_to_mat4(tf)
            self.get_logger().info(
                f"reloc compose: using TF {self._odom_frame}->{child} "
                f"xy=({mat[0,3]:.2f},{mat[1,3]:.2f})"
            )
            return mat
        try:
            t_ob = self._buffer.lookup_transform(
                self._odom_frame, self._base_frame, times[0], timeout=dur
            )
            t_bl = self._buffer.lookup_transform(
                self._base_frame, "livox_frame", times[0], timeout=dur
            )
            mat = _stamp_to_mat4(t_ob) @ _stamp_to_mat4(t_bl)
            self.get_logger().info("reloc compose: using odom->base->livox chain")
            return mat
        except Exception:
            return None

    def _compose_map_to_odom_from_icp(
        self, msg: PoseWithCovarianceStamped
    ) -> np.ndarray | None:
        """ICP 位姿为 T_map_sensor；合成 T_map_odom = T_map_sensor @ inv(T_odom_sensor)。"""
        t_map_sensor = _pose_to_mat4(msg.pose.pose)
        timeout = float(self.get_parameter("tf_timeout").value)
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        t_odom_sensor = self._lookup_t_odom_sensor(timeout, stamp)
        if t_odom_sensor is None:
            self.get_logger().error(
                "reloc: 无 odom->sensor TF。请确认 FAST-LIO 在发 /lio_odom_raw，"
                "且重定位前 odom->base 已存在；未更新 map->odom。"
            )
            return None

        t_sensor_odom = np.linalg.inv(t_odom_sensor)
        return t_map_sensor @ t_sensor_odom

    def _is_spin_protected(self) -> bool:
        now = time.monotonic()
        if now < self._spin_settle_until:
            return True
        if now - self._last_odom_mono > 0.6:
            return False
        if abs(self._last_wz) >= self._spin_block_wz:
            self._spin_settle_until = now + self._spin_settle_sec
            return True
        return False

    def _refine_jump_ok(self, t_map_odom: np.ndarray) -> bool:
        if self._t_map_odom_mat is None:
            return True
        t_old = self._t_map_odom_mat
        dx = float(t_map_odom[0, 3] - t_old[0, 3])
        dy = float(t_map_odom[1, 3] - t_old[1, 3])
        if math.hypot(dx, dy) > self._refine_max_xy:
            return False
        yaw_old = math.atan2(t_old[1, 0], t_old[0, 0])
        yaw_new = math.atan2(t_map_odom[1, 0], t_map_odom[0, 0])
        return abs(_normalize_angle(yaw_new - yaw_old)) <= self._refine_max_yaw

    def _on_reloc_pose(self, msg: PoseWithCovarianceStamped) -> None:
        latch = bool(self.get_parameter("latch_map_to_odom").value)
        allow_refine = bool(self.get_parameter("allow_icp_refine").value)
        if self._reloc_applied and latch and not allow_refine:
            return
        if self._reloc_applied and self._is_spin_protected():
            self.get_logger().warn(
                "转圈中/刚停转：跳过 map->odom ICP 更新，避免 360° 后定位飘",
                throttle_duration_sec=2.0,
            )
            return
        if self._reloc_applied and latch and allow_refine:
            min_iv = float(self.get_parameter("icp_refine_min_interval_sec").value)
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_icp_refine_time < min_iv:
                return
            self._last_icp_refine_time = now
        t_map_odom = self._compose_map_to_odom_from_icp(msg)
        if t_map_odom is None:
            return
        if self._reloc_applied and not self._refine_jump_ok(t_map_odom):
            self.get_logger().warn(
                "ICP refine 跳变过大（转圈误匹配），已拒绝更新 map->odom",
                throttle_duration_sec=2.0,
            )
            return
        self._yaw_filt = None
        prefix = "ICP refine" if self._reloc_applied else "Relocalization"
        self._apply_map_to_odom_mat(t_map_odom, prefix)
        if prefix == "ICP refine":
            self.get_logger().debug(
                f"ICP refine map->odom applied (interval>="
                f"{float(self.get_parameter('icp_refine_min_interval_sec').value):.1f}s)"
            )

    def _smooth_yaw(self, yaw: float, wz: float) -> float:
        if not self._planar_yaw or self._yaw_alpha <= 0.0:
            return yaw
        spinning = abs(wz) >= self._spin_wz_th
        # 转圈时不平滑：避免 /transformed_cloud 与 map 系点云“跟着狗转、停转才慢慢回正”
        if spinning:
            self._was_spinning = True
            self._yaw_filt = yaw
            return yaw
        if self._was_spinning:
            self._was_spinning = False
            self._yaw_filt = yaw
            return yaw
        if self._yaw_filt is None:
            self._yaw_filt = yaw
            return yaw
        dy = _normalize_angle(yaw - self._yaw_filt)
        self._yaw_filt = _normalize_angle(self._yaw_filt + self._yaw_alpha * dy)
        return self._yaw_filt

    def _lio_pose_to_base(self, msg: Odometry) -> Pose:
        """LIO 位姿在 imu_link；与 imu 对齐时直接用作 odom->base，勿经 livox 链外参。"""
        child = (msg.child_frame_id or "imu_link").strip()
        if not self._apply_lio_extrinsic or child in (
            self._base_frame,
            "imu_link",
        ):
            return msg.pose.pose
        dur = rclpy.duration.Duration(seconds=float(self.get_parameter("tf_timeout").value))
        for stamp in (msg.header.stamp, rclpy.time.Time()):
            try:
                t_bc = self._buffer.lookup_transform(
                    self._base_frame, child, stamp, timeout=dur
                )
                t = t_bc.transform.translation
                r = t_bc.transform.rotation
                mat_bc = np.eye(4, dtype=np.float64)
                mat_bc[:3, :3] = _quat_to_rot(r.x, r.y, r.z, r.w)
                mat_bc[0, 3] = t.x
                mat_bc[1, 3] = t.y
                mat_bc[2, 3] = t.z
                return _mat4_to_pose(_pose_to_mat4(msg.pose.pose) @ np.linalg.inv(mat_bc))
            except Exception:
                continue
        return msg.pose.pose

    def _publish_map_state_estimation(self, msg: Odometry, pose_odom_base: Pose) -> None:
        """T_map_base = T_map_odom @ T_odom_base，供 EGO grid_map（map 系，~10Hz）。"""
        if self._se_pub is None or not self._have_map_odom or self._t_map_odom_mat is None:
            return
        t_map_base = self._t_map_odom_mat @ _pose_to_mat4(pose_odom_base)
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._map_frame
        out.child_frame_id = self._base_frame
        out.pose.pose = _mat4_to_pose(t_map_base)
        out.twist = msg.twist
        self._se_pub.publish(out)

    def _on_odom(self, msg: Odometry) -> None:
        if not self.get_parameter("publish_odom_to_base").value:
            return
        self._odom_from_topic = True
        self._last_wz = float(msg.twist.twist.angular.z)
        self._last_odom_mono = time.monotonic()
        pose = self._lio_pose_to_base(msg)
        if self._planar_yaw:
            o = pose.orientation
            yaw = _yaw_from_pose_quat(o.x, o.y, o.z, o.w)
            wz = float(msg.twist.twist.angular.z)
            yaw = self._smooth_yaw(yaw, wz)
            qx, qy, qz, qw = _rpy_to_quat(0.0, 0.0, yaw)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
        if self._lock_z:
            pose.position.z = self._z_fixed
        self._odom_to_base.header.stamp = msg.header.stamp
        self._odom_to_base.transform.translation.x = pose.position.x
        self._odom_to_base.transform.translation.y = pose.position.y
        self._odom_to_base.transform.translation.z = pose.position.z
        self._odom_to_base.transform.rotation = pose.orientation
        self._have_odom_base = True
        self._tf_pub.sendTransform([self._odom_to_base])

        if self._se_pub is not None:
            self._publish_map_state_estimation(msg, pose)

    def _tick(self) -> None:
        now = self.get_clock().now().to_msg()
        # 勿用 now() 重发 odom->base（位姿来自 ~10Hz LIO），否则 RViz 里 base_link 相对 imu_link 明显滞后
        if (
            self._republish_odom_timer
            and self._have_odom_base
            and self.get_parameter("publish_odom_to_base").value
            and (self._odom_from_topic or bool(
                self.get_parameter("seed_odom_to_base_identity").value
            ))
        ):
            self._odom_to_base.header.stamp = now
            self._tf_pub.sendTransform([self._odom_to_base])
        if (
            self._refresh_map_stamp
            and self._reloc_applied
            and self._have_map_odom
            and self.get_parameter("publish_map_to_odom").value
        ):
            self._map_to_odom.header.stamp = now
            self._map_static_pub.sendTransform([self._map_to_odom])

        timeout = float(self.get_parameter("tf_timeout").value)
        dur = rclpy.duration.Duration(seconds=timeout)
        odom_ok = False
        map_ok = False
        try:
            self._buffer.lookup_transform(
                self._odom_frame, self._base_frame, rclpy.time.Time(), timeout=dur
            )
            odom_ok = True
        except Exception:
            odom_ok = False

        need_map = bool(self.get_parameter("publish_map_to_odom").value)
        if need_map and self._reloc_applied and self._have_map_odom:
            try:
                self._buffer.lookup_transform(
                    self._map_frame, self._base_frame, rclpy.time.Time(), timeout=dur
                )
                map_ok = True
            except Exception:
                map_ok = False
        elif need_map and not self._have_map_odom:
            map_ok = False
        else:
            map_ok = odom_ok

        ok = odom_ok and (map_ok if (need_map and self._have_map_odom) else True)

        self._tf_ok_pub.publish(Bool(data=ok))
        if not odom_ok:
            status = "TF_LOST"
        elif need_map and not self._have_map_odom:
            status = "MAP_ODOM_WAIT"
        elif need_map and not map_ok:
            status = "TF_LOST"
        else:
            status = "TF_OK"
        self._health_pub.publish(String(data=status))


def main() -> None:
    rclpy.init()
    node = TfManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
