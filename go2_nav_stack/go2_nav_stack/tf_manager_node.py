#!/usr/bin/env python3
"""TF 监视与 map->odom 保持；静态外参发布；odom 健康检测。

ICP /icp_result 为 T_map_lidar（当前扫帧 livox -> map），须结合重定位时刻的
T_odom_lidar 合成 T_map_odom = T_map_lidar @ inv(T_odom_lidar)，不能把 ICP 位姿
直接当作 map->odom。
"""

from __future__ import annotations

import math

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
        self.declare_parameter("publish_odom_to_base", True)
        self.declare_parameter("publish_map_to_odom", True)
        self.declare_parameter("latch_map_to_odom", True)
        self.declare_parameter("seed_map_to_odom_identity", True)
        self.declare_parameter("odom_topic", "/state_estimation")

        self._map_frame = self.get_parameter("map_frame").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._lidar_frame = self.get_parameter("lidar_frame").value
        self._map_to_odom = TransformStamped()
        self._map_to_odom.header.frame_id = self._map_frame
        self._map_to_odom.child_frame_id = self._odom_frame
        self._have_map_odom = False
        self._reloc_applied = False
        self._odom_to_base = TransformStamped()
        self._odom_to_base.header.frame_id = self._odom_frame
        self._odom_to_base.child_frame_id = self._base_frame
        self._have_odom_base = False

        self._tf_pub = TransformBroadcaster(self)
        self._static_pub = StaticTransformBroadcaster(self)
        self._map_static_pub = StaticTransformBroadcaster(self)
        self._tf_ok_pub = self.create_publisher(Bool, "/navigation/tf_ok", 10)
        self._health_pub = self.create_publisher(String, "/navigation/health", 10)

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)

        odom_topic = self.get_parameter("odom_topic").value
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/relocalization/pose",
            self._on_reloc_pose,
            10,
        )

        self._publish_static()
        if bool(self.get_parameter("seed_map_to_odom_identity").value):
            self._seed_map_to_odom_identity()
        period = 1.0 / max(1.0, float(self.get_parameter("publish_rate").value))
        self.create_timer(period, self._tick)
        self.get_logger().info(
            "tf_manager_node: odom->base_link from /state_estimation (KISS); "
            f"map->odom static from /relocalization/pose "
            f"(T_map_odom=T_map_{self._lidar_frame}@inv(T_odom_{self._lidar_frame})); "
            "KISS publish_odom_tf should be false"
        )

    def _publish_static(self) -> None:
        imu = TransformStamped()
        imu.header.stamp = self.get_clock().now().to_msg()
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

        lidar = TransformStamped()
        lidar.header.stamp = imu.header.stamp
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
        self._static_pub.sendTransform([imu, lidar])

    def _apply_map_to_odom_mat(self, t_map_odom: np.ndarray, log_prefix: str) -> None:
        pose = _mat4_to_pose(t_map_odom)
        self._map_to_odom.header.stamp = self.get_clock().now().to_msg()
        self._map_to_odom.header.frame_id = self._map_frame
        self._map_to_odom.child_frame_id = self._odom_frame
        self._map_to_odom.transform.translation.x = pose.position.x
        self._map_to_odom.transform.translation.y = pose.position.y
        self._map_to_odom.transform.translation.z = pose.position.z
        self._map_to_odom.transform.rotation = pose.orientation
        self._have_map_odom = True
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

    def _seed_map_to_odom_identity(self) -> None:
        """重定位前占位：彩色点云与 odom 一致，但与先验 PCD 不会对齐。"""
        mat = np.eye(4, dtype=np.float64)
        self._apply_map_to_odom_mat(mat, "Seeded identity")
        self.get_logger().warn(
            "Prior map (/prior_map) will NOT overlap live cloud until ScanContext+ICP reloc."
        )

    def _compose_map_to_odom_from_icp(self, msg: PoseWithCovarianceStamped) -> np.ndarray:
        """ICP 位姿为 T_map_lidar；合成 T_map_odom = T_map_lidar @ inv(T_odom_lidar)。"""
        t_map_lidar = _pose_to_mat4(msg.pose.pose)
        timeout = float(self.get_parameter("tf_timeout").value)
        dur = rclpy.duration.Duration(seconds=timeout)
        try:
            t_odom_lidar = self._buffer.lookup_transform(
                self._odom_frame,
                self._lidar_frame,
                rclpy.time.Time(),
                timeout=dur,
            )
        except Exception as ex:
            self.get_logger().warn(
                f"reloc: TF {self._odom_frame}->{self._lidar_frame} failed ({ex}); "
                "fallback: use ICP pose as map->odom (may misalign prior map)"
            )
            return t_map_lidar

        t_odom_lidar_mat = _stamp_to_mat4(t_odom_lidar)
        t_lidar_odom = np.linalg.inv(t_odom_lidar_mat)
        t_map_odom = t_map_lidar @ t_lidar_odom

        dx = float(t_odom_lidar_mat[0, 3])
        dy = float(t_odom_lidar_mat[1, 3])
        self.get_logger().info(
            f"reloc compose: odom->{self._lidar_frame} offset=({dx:.2f},{dy:.2f})"
        )
        return t_map_odom

    def _on_reloc_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if self._reloc_applied and bool(self.get_parameter("latch_map_to_odom").value):
            return
        t_map_odom = self._compose_map_to_odom_from_icp(msg)
        self._apply_map_to_odom_mat(t_map_odom, "Relocalization")

    def _on_odom(self, msg: Odometry) -> None:
        if not self.get_parameter("publish_odom_to_base").value:
            return
        self._odom_to_base.header.stamp = msg.header.stamp
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._odom_to_base.transform.translation.x = p.x
        self._odom_to_base.transform.translation.y = p.y
        self._odom_to_base.transform.translation.z = p.z
        self._odom_to_base.transform.rotation = o
        self._have_odom_base = True
        self._odom_to_base.header.stamp = self.get_clock().now().to_msg()
        self._tf_pub.sendTransform([self._odom_to_base])

    def _tick(self) -> None:
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
        if need_map and self._have_map_odom:
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
