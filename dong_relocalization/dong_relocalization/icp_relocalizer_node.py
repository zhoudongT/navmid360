"""ICP relocalization: prior PCD (map frame) + live PointCloud2, service trigger."""

from __future__ import annotations

import threading
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from tf2_sensor_msgs import do_transform_cloud

try:
    import open3d as o3d
except ImportError as exc:
    o3d = None  # type: ignore
    _OPEN3D_ERR = exc
else:
    _OPEN3D_ERR = None


def _msg_to_o3d(cloud: PointCloud2) -> o3d.geometry.PointCloud:
    pts = []
    from sensor_msgs_py import point_cloud2

    for x, y, z in point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
        if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
            pts.append([float(x), float(y), float(z)])
    pcd = o3d.geometry.PointCloud()
    if pts:
        pcd.points = o3d.utility.Vector3dVector(np.asarray(pts, dtype=np.float64))
    return pcd


def _mat44_to_pose_msg(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return position (3,) and quaternion (x,y,z,w) from 4x4."""
    R = T[:3, :3]
    t = T[:3, 3]
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    q /= np.linalg.norm(q) + 1e-12
    return t, q


class IcpRelocalizerNode(Node):
    def __init__(self) -> None:
        super().__init__("icp_relocalizer")
        if o3d is None:
            raise RuntimeError(
                "需要 Python 包 open3d（例如 pip install open3d）。原始错误: " + str(_OPEN3D_ERR)
            )

        self.declare_parameter("prior_pcd_path", "")
        self.declare_parameter("scan_topic", "/cloud_registered")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("lidar_frame", "camera_init")
        self.declare_parameter("voxel_size", 0.25)
        self.declare_parameter("crop_margin_m", 8.0)
        self.declare_parameter("max_correspondence_distance", 0.8)
        self.declare_parameter("icp_max_iterations", 40)
        self.declare_parameter("min_fitness", 0.08)
        self.declare_parameter("max_rmse", 0.35)
        self.declare_parameter("publish_tf", False)

        self._prior: Optional[o3d.geometry.PointCloud] = None
        self._prior_lock = threading.Lock()
        self._latest_scan: Optional[PointCloud2] = None
        self._scan_lock = threading.Lock()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_broadcaster: Optional[TransformBroadcaster] = None
        if bool(self.get_parameter("publish_tf").value):
            self._tf_broadcaster = TransformBroadcaster(self)

        self._load_prior()

        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.create_subscription(PointCloud2, scan_topic, self._on_scan, 1)

        self._pose_pub = self.create_publisher(PoseStamped, "/relocalization/lidar_pose_in_map", 1)
        self._pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/relocalization/lidar_pose_in_map_cov", 1
        )
        self.create_service(Trigger, "~/trigger_icp", self._on_trigger)

        self.get_logger().info(
            f"icp_relocalizer: prior loaded, scan={scan_topic}, "
            f"map={self.get_parameter('map_frame').value}, "
            f"lidar={self.get_parameter('lidar_frame').value}. Call ~/trigger_icp"
        )

    def _load_prior(self) -> None:
        path = str(self.get_parameter("prior_pcd_path").value).strip()
        if not path:
            raise RuntimeError("参数 prior_pcd_path 不能为空")
        pcd = o3d.io.read_point_cloud(path)
        if not pcd.has_points() or len(pcd.points) == 0:
            raise RuntimeError(f"先验 PCD 为空或无法读取: {path}")
        vs = float(self.get_parameter("voxel_size").value)
        if vs > 1e-6:
            pcd = pcd.voxel_down_sample(vs)
        with self._prior_lock:
            self._prior = pcd
        self.get_logger().info(f"先验地图已加载并体素下采样: {path} 点数={len(pcd.points)}")

    def _on_scan(self, msg: PointCloud2) -> None:
        with self._scan_lock:
            self._latest_scan = msg

    def _on_trigger(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        with self._scan_lock:
            scan_msg = self._latest_scan
        if scan_msg is None:
            resp.success = False
            resp.message = "尚未收到点云，无法重定位。"
            return resp

        map_f = self.get_parameter("map_frame").get_parameter_value().string_value
        lidar_f = self.get_parameter("lidar_frame").get_parameter_value().string_value

        try:
            tf = self.tf_buffer.lookup_transform(map_f, lidar_f, rclpy.time.Time())
        except Exception as ex:
            resp.success = False
            resp.message = f"TF {map_f}<-{lidar_f} 查询失败: {ex}"
            return resp

        try:
            cloud_map = do_transform_cloud(scan_msg, tf)
        except Exception as ex:
            resp.success = False
            resp.message = f"点云变换到 {map_f} 失败: {ex}"
            return resp

        scan_o3 = _msg_to_o3d(cloud_map)
        if len(scan_o3.points) < 200:
            resp.success = False
            resp.message = f"当前帧在 {map_f} 下点数过少 ({len(scan_o3.points)}), 无法 ICP。"
            return resp

        vs = float(self.get_parameter("voxel_size").value)
        if vs > 1e-6:
            scan_o3 = scan_o3.voxel_down_sample(vs)

        with self._prior_lock:
            if self._prior is None:
                resp.success = False
                resp.message = "先验地图未加载。"
                return resp
            prior_full = self._prior

        pts = np.asarray(scan_o3.points)
        c = np.mean(pts, axis=0)
        margin = float(self.get_parameter("crop_margin_m").value)
        mn = c - margin
        mx = c + margin
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=mn, max_bound=mx)
        prior_crop = prior_full.crop(bbox)
        if len(prior_crop.points) < 500:
            prior_crop = prior_full

        max_dist = float(self.get_parameter("max_correspondence_distance").value)
        max_iter = int(self.get_parameter("icp_max_iterations").value)
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)

        reg = o3d.pipelines.registration.registration_icp(
            scan_o3,
            prior_crop,
            max_dist,
            np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria,
        )

        fitness = float(reg.fitness)
        rmse = float(reg.inlier_rmse)
        min_fit = float(self.get_parameter("min_fitness").value)
        max_rmse = float(self.get_parameter("max_rmse").value)
        if fitness < min_fit or rmse > max_rmse:
            resp.success = False
            resp.message = (
                f"ICP 质量不足: fitness={fitness:.4f} (min {min_fit}), rmse={rmse:.4f} (max {max_rmse})。"
            )
            return resp

        T_delta = np.asarray(reg.transformation, dtype=np.float64)
        T_ml = self._tf_msg_to_mat44(tf.transform)
        T_new = T_delta @ T_ml

        now = self.get_clock().now().to_msg()
        t, qv = _mat44_to_pose_msg(T_new)

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = map_f
        pose.pose.position.x = float(t[0])
        pose.pose.position.y = float(t[1])
        pose.pose.position.z = float(t[2])
        pose.pose.orientation.x = float(qv[0])
        pose.pose.orientation.y = float(qv[1])
        pose.pose.orientation.z = float(qv[2])
        pose.pose.orientation.w = float(qv[3])
        self._pose_pub.publish(pose)

        cov = PoseWithCovarianceStamped()
        cov.header = pose.header
        cov.pose.pose = pose.pose
        cov.pose.covariance[0] = 0.05
        cov.pose.covariance[7] = 0.05
        cov.pose.covariance[14] = 0.05
        cov.pose.covariance[35] = 0.02
        self._pose_cov_pub.publish(cov)

        if self._tf_broadcaster is not None:
            ts = TransformStamped()
            ts.header.stamp = now
            ts.header.frame_id = map_f
            ts.child_frame_id = lidar_f
            ts.transform.translation.x = float(t[0])
            ts.transform.translation.y = float(t[1])
            ts.transform.translation.z = float(t[2])
            ts.transform.rotation.x = float(qv[0])
            ts.transform.rotation.y = float(qv[1])
            ts.transform.rotation.z = float(qv[2])
            ts.transform.rotation.w = float(qv[3])
            self._tf_broadcaster.sendTransform(ts)

        resp.success = True
        resp.message = (
            f"ICP 成功: fitness={fitness:.4f}, rmse={rmse:.4f}. "
            f"已发布 {map_f} 下 {lidar_f} 位姿到 /relocalization/lidar_pose_in_map"
        )
        self.get_logger().info(resp.message)
        return resp

    @staticmethod
    def _tf_msg_to_mat44(t) -> np.ndarray:
        """geometry_msgs/Transform -> 4x4 (numpy)."""
        from math import cos, sin

        x, y, z = t.translation.x, t.translation.y, t.translation.z
        qx, qy, qz, qw = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        R = np.array(
            [
                [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
                [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
                [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
            ],
            dtype=np.float64,
        )
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T


def main() -> None:
    rclpy.init()
    try:
        node = IcpRelocalizerNode()
    except RuntimeError as e:
        print(f"icp_relocalizer 启动失败: {e}")
        rclpy.shutdown()
        raise SystemExit(1) from e
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
