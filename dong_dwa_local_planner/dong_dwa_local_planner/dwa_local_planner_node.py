# Copyright (c) 2026, adapted for dong_3d_nav.
# DWA logic from MoveBase3D local_planner_node.py (ROS2 script in upstream repo).

"""ROS2 node: subscribe global Path + optional obstacle cloud, publish cmd_vel via DWA."""

from __future__ import annotations

import math
import threading
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from dong_dwa_local_planner.dwa_core import DWAConfig, plan_velocity_dwa, snap_obstacles_xy


def _quat_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(math.atan2(siny_cosp, cosy_cosp))


class DWALocalPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("dwa_local_planner")

        self.declare_parameter("enabled", False)
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("obstacle_cloud_topic", "")
        self.declare_parameter("cmd_vel_out_topic", "/cmd_vel_dwa")
        self.declare_parameter("local_path_topic", "/local_path_dwa")
        self.declare_parameter("global_path_step", 10)
        self.declare_parameter("obstacle_cell_snap", 0.3)
        self.declare_parameter("reached_position_tolerance", 0.25)
        self.declare_parameter("reached_yaw_tolerance", 0.2)
        self.declare_parameter("goal_min_angular_speed", -0.5)
        self.declare_parameter("goal_max_angular_speed", 0.5)
        self.declare_parameter("goal_angular_gain", 2.0)
        self.declare_parameter("predict_time", 2.0)
        self.declare_parameter("dwa_lookahead_step", 0.1)
        self.declare_parameter("goal_weight", 1.0)
        self.declare_parameter("speed_weight", 0.5)
        self.declare_parameter("obstacle_weight", 1.9)
        self.declare_parameter("safe_distance", 0.3)
        self.declare_parameter("omega_min", -1.0)
        self.declare_parameter("omega_max", 1.0)
        self.declare_parameter("v_min", -0.2)
        self.declare_parameter("v_max", 0.6)
        self.declare_parameter("thread_workers", 8)
        self.declare_parameter("control_period", 0.1)

        self._lock = threading.Lock()
        self._path_xyz_yaw: Optional[np.ndarray] = None  # (N, 4)
        self._obstacle_xy: np.ndarray = np.zeros((0, 2), dtype=np.float64)
        self._goal_position_reached = False
        self._goal_yaw_reached = False
        self._goal_yaw: float = 0.0
        self._cur_yaw: float = 0.0
        self._best_control = (0.0, 0.0)
        self._control_cmd = Twist()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        path_topic = self.get_parameter("path_topic").get_parameter_value().string_value
        self.create_subscription(Path, path_topic, self._on_path, 1)

        obs_topic = self.get_parameter("obstacle_cloud_topic").get_parameter_value().string_value.strip()
        if obs_topic:
            self.create_subscription(PointCloud2, obs_topic, self._on_obstacle_cloud, 2)

        self.create_subscription(Bool, "~/enable", self._on_enable, 10)

        cmd_topic = self.get_parameter("cmd_vel_out_topic").get_parameter_value().string_value
        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        lp_topic = self.get_parameter("local_path_topic").get_parameter_value().string_value
        self._local_path_pub = self.create_publisher(Path, lp_topic, 1)

        period = float(self.get_parameter("control_period").value)
        period = max(0.02, period)
        self.create_timer(period, self._control_tick)

        self._enabled = bool(self.get_parameter("enabled").value)
        self.get_logger().info(
            f"dwa_local_planner ready. enabled={self._enabled} path={path_topic} cmd={cmd_topic}"
        )

    def _on_enable(self, msg: Bool) -> None:
        self._enabled = bool(msg.data)
        self.get_logger().info(f"DWA enable set to {self._enabled}")

    def _on_path(self, msg: Path) -> None:
        if not msg.poses:
            with self._lock:
                self._path_xyz_yaw = None
            return
        rows = []
        for ps in msg.poses:
            p = ps.pose.position
            o = ps.pose.orientation
            yaw = _quat_yaw(o.x, o.y, o.z, o.w)
            rows.append((float(p.x), float(p.y), float(p.z), yaw))
        arr = np.asarray(rows, dtype=np.float64)
        with self._lock:
            self._path_xyz_yaw = arr
            self._goal_position_reached = False
            self._goal_yaw_reached = False
            gx, gy, _, gyaw = arr[-1]
            self._goal_yaw = gyaw

    def _on_obstacle_cloud(self, msg: PointCloud2) -> None:
        pts = []
        try:
            for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                if math.isfinite(x) and math.isfinite(y):
                    pts.append((float(x), float(y)))
        except Exception as exc:
            self.get_logger().warn(f"obstacle cloud parse failed: {exc}")
            return
        cell = float(self.get_parameter("obstacle_cell_snap").value)
        cell = max(1e-3, cell)
        snapped = snap_obstacles_xy(pts, cell)
        with self._lock:
            self._obstacle_xy = snapped

    def _lookup_robot(self) -> Optional[tuple[float, float, float, float]]:
        map_f = self.get_parameter("map_frame_id").get_parameter_value().string_value
        base_f = self.get_parameter("base_frame_id").get_parameter_value().string_value
        try:
            tf = self.tf_buffer.lookup_transform(map_f, base_f, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = _quat_yaw(q.x, q.y, q.z, q.w)
        return float(t.x), float(t.y), float(t.z), yaw

    def _find_closest_path_index(self, px: float, py: float, path: np.ndarray) -> int:
        dx = path[:, 0] - px
        dy = path[:, 1] - py
        d2 = dx * dx + dy * dy
        return int(np.argmin(d2))

    def _choose_goal_window(self, path: np.ndarray, closest_idx: int) -> np.ndarray:
        """Returns (K, 3) xyz goal window; last row is DWA goal xy reference (MoveBase3D style)."""
        k = int(self.get_parameter("global_path_step").value)
        k = max(1, k)
        n = path.shape[0]
        out = np.zeros((k, 3), dtype=np.float64)
        for i in range(k):
            idx = min(n - 1, closest_idx + i)
            out[i, 0] = path[idx, 0]
            out[i, 1] = path[idx, 1]
            out[i, 2] = path[idx, 2]
        return out

    def _distance_to_final(self, px: float, py: float, path: np.ndarray) -> float:
        gx, gy = float(path[-1, 0]), float(path[-1, 1])
        return float(math.hypot(px - gx, py - gy))

    def _control_tick(self) -> None:
        if not self._enabled:
            return

        pose = self._lookup_robot()
        if pose is None:
            return
        px, py, pz, yaw = pose
        self._cur_yaw = yaw

        with self._lock:
            path = self._path_xyz_yaw.copy() if self._path_xyz_yaw is not None else None
            obstacles = self._obstacle_xy.copy()

        if path is None or path.shape[0] < 2:
            self._publish_zero()
            return

        pos_tol = float(self.get_parameter("reached_position_tolerance").value)
        if self._distance_to_final(px, py, path) <= pos_tol:
            self._goal_position_reached = True

        if self._goal_position_reached and self._goal_yaw_reached:
            self._publish_zero()
            return

        if self._goal_position_reached:
            self._publish_yaw_only()
            return

        closest = self._find_closest_path_index(px, py, path)
        window = self._choose_goal_window(path, closest)
        goal_x, goal_y = float(window[-1, 0]), float(window[-1, 1])

        cfg = DWAConfig(
            predict_time=float(self.get_parameter("predict_time").value),
            dt=float(self.get_parameter("dwa_lookahead_step").value),
            safe_distance=float(self.get_parameter("safe_distance").value),
            goal_weight=float(self.get_parameter("goal_weight").value),
            speed_weight=float(self.get_parameter("speed_weight").value),
            obstacle_weight=float(self.get_parameter("obstacle_weight").value),
            v_min=float(self.get_parameter("v_min").value),
            v_max=float(self.get_parameter("v_max").value),
            omega_min=float(self.get_parameter("omega_min").value),
            omega_max=float(self.get_parameter("omega_max").value),
            thread_workers=int(self.get_parameter("thread_workers").value),
        )

        traj, (bv, bw), ok = plan_velocity_dwa(
            px,
            py,
            yaw,
            goal_x,
            goal_y,
            obstacles,
            self._control_cmd.linear.x,
            self._control_cmd.angular.z,
            cfg,
        )
        if ok and traj is not None:
            self._best_control = (bv, bw)
            self._publish_local_path(traj, pz)
        else:
            self._best_control = (0.0, 0.0)

        self._control_cmd.linear.x = self._best_control[0]
        self._control_cmd.linear.y = 0.0
        self._control_cmd.angular.z = self._best_control[1]
        self._cmd_pub.publish(self._control_cmd)

    def _publish_local_path(self, trajectory: np.ndarray, z_ref: float) -> None:
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.get_parameter("map_frame_id").get_parameter_value().string_value
        for i in range(trajectory.shape[0]):
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x = float(trajectory[i, 0])
            ps.pose.position.y = float(trajectory[i, 1])
            ps.pose.position.z = float(z_ref) + 0.1
            ps.pose.orientation.w = 1.0
            out.poses.append(ps)
        self._local_path_pub.publish(out)

    def _publish_zero(self) -> None:
        z = Twist()
        self._cmd_pub.publish(z)
        self._control_cmd = z

    def _publish_yaw_only(self) -> None:
        yaw_tol = float(self.get_parameter("reached_yaw_tolerance").value)
        ydiff = (self._goal_yaw - self._cur_yaw + math.pi) % (2.0 * math.pi) - math.pi
        cmd = Twist()
        if abs(ydiff) > yaw_tol:
            g = float(self.get_parameter("goal_angular_gain").value)
            w = max(
                float(self.get_parameter("goal_min_angular_speed").value),
                min(float(self.get_parameter("goal_max_angular_speed").value), g * ydiff),
            )
            cmd.angular.z = w
        elif abs(ydiff) > 0.05:
            cmd.angular.z = 0.1 * (1.0 if ydiff > 0 else -1.0)
        else:
            self._goal_yaw_reached = True
        self._cmd_pub.publish(cmd)
        self._control_cmd = cmd


def main() -> None:
    rclpy.init()
    node = DWALocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
