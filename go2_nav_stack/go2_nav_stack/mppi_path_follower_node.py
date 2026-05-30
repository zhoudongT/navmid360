#!/usr/bin/env python3
"""将 dong_path 全局路径 /planned_path 送入 Nav2 controller_server 的 FollowPath action。

流程:
  map_viewer / 网页 -> /planned_path + /start_navigation
  -> mppi_path_follower -> FollowPath(controller_id=FollowPath)
  -> MPPI 局部规划 + local_costmap 避障 -> /cmd_vel
"""

from __future__ import annotations

import copy
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
import tf2_ros
from tf2_ros import TransformException


class MppiPathFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("mppi_path_follower")
        self.declare_parameter("planned_path_topic", "/global_path_processed")
        self.declare_parameter("fallback_planned_path_topic", "/planned_path")
        self.declare_parameter("start_navigation_topic", "/start_navigation")
        self.declare_parameter("stop_navigation_topic", "/stop_navigation")
        self.declare_parameter("follow_path_action", "/controller_server/follow_path")
        self.declare_parameter("follow_path_action_alt", "/follow_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "general_goal_checker")
        self.declare_parameter("require_start_command", True)
        self.declare_parameter("require_reloc_ok", True)
        # 保留 dong_path 写入的各点 yaw，并轻度平滑，供 MPPI PathAlign 参考
        self.declare_parameter("preserve_global_path_yaw", True)
        self.declare_parameter("smooth_path_yaw", False)
        self.declare_parameter("yaw_smooth_window", 3)
        self.declare_parameter("yaw_lookahead_m", 0.7)
        self.declare_parameter("downsample_min_spacing_m", 0.15)
        self.declare_parameter("retry_on_abort", True)
        self.declare_parameter("enable_escape_recovery", True)
        self.declare_parameter("max_recovery_attempts", 4)
        self.declare_parameter("escape_min_dist_to_goal_m", 2.5)
        self.declare_parameter("escape_min_follow_sec", 2.0)
        self.declare_parameter("soft_abort_retry_max", 12)
        self.declare_parameter("soft_abort_retry_pause_sec", 1.5)
        self.declare_parameter("backup_speed", 0.2)
        self.declare_parameter("backup_duration_sec", 1.0)
        self.declare_parameter("rotate_speed", 0.55)
        self.declare_parameter("rotate_duration_sec", 1.2)
        self.declare_parameter("recovery_pause_sec", 0.4)
        self.declare_parameter("enable_pre_align", True)
        self.declare_parameter("pre_align_threshold_deg", 45.0)
        self.declare_parameter("pre_align_tolerance_deg", 12.0)
        self.declare_parameter("pre_align_wz", 0.45)
        self.declare_parameter("pre_align_timeout_sec", 20.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("base_frame_alt", "base_footprint")
        self.declare_parameter("tf_timeout_sec", 0.35)

        self._path: Path | None = None
        self._goal_handle = None
        self._navigating = False
        self._bringup_status = "PENDING"
        self._pending_start = False
        self._recovering = False
        self._recovery_phase = ""
        self._recovery_attempts = 0
        self._soft_abort_retries = 0
        self._last_dist_to_goal: float | None = None
        self._goal_accept_mono: float | None = None
        self._rotate_sign = 1.0
        self._recovery_phase_end = None
        self._retry_timer = None
        self._reloc_ok = False
        self._path_from_processed = False
        self._pre_aligning = False
        self._pre_align_target_yaw = 0.0
        self._prepared_path_pending: Path | None = None
        self._pre_align_started_at = None
        self._last_mppi_cmd_time = None
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._base_frame_alt = str(self.get_parameter("base_frame_alt").value).strip()
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._action_name = str(self.get_parameter("follow_path_action").value)

        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_pub = self.create_publisher(String, "/local_planner/status", 10)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        proc_topic = str(self.get_parameter("planned_path_topic").value)
        fallback_topic = str(self.get_parameter("fallback_planned_path_topic").value)
        self.create_subscription(Path, proc_topic, self._on_processed_path, path_qos)
        if fallback_topic and fallback_topic != proc_topic:
            self.create_subscription(
                Path, fallback_topic, self._on_fallback_path, path_qos
            )
        self.create_subscription(
            Bool,
            self.get_parameter("start_navigation_topic").value,
            self._on_start,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("stop_navigation_topic").value,
            self._on_stop,
            10,
        )
        self.create_subscription(Bool, "/relocalization/ok", self._on_reloc_ok, 10)
        self.create_subscription(String, "/relocalization/status", self._on_reloc_status, 10)

        self._client = ActionClient(self, FollowPath, self._action_name)
        self._alt_action = str(self.get_parameter("follow_path_action_alt").value).strip()
        self._alt_client: ActionClient | None = None
        if self._alt_action and self._alt_action != self._action_name:
            self._alt_client = ActionClient(self, FollowPath, self._alt_action)
        self.create_subscription(String, "/nav2_controller/status", self._on_bringup_status, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_timer(0.5, self._on_timer)
        self.create_timer(0.05, self._on_recovery_timer)
        self.create_timer(2.0, self._on_action_probe)
        self.get_logger().info(
            f"mppi_path_follower: {proc_topic}"
            f"{f' (+fallback {fallback_topic})' if fallback_topic != proc_topic else ''} "
            f"-> FollowPath({self._action_name}"
            f"{f' | alt {self._alt_action}' if self._alt_client else ''})"
        )

    def _on_reloc_ok(self, msg: Bool) -> None:
        if msg.data:
            self._reloc_ok = True

    def _on_reloc_status(self, msg: String) -> None:
        if msg.data.strip() == "RELOCALIZED":
            self._reloc_ok = True

    def _on_action_probe(self) -> None:
        if self._client.server_is_ready():
            return
        if self._alt_client is not None and self._alt_client.server_is_ready():
            self.get_logger().info(
                f"FollowPath 使用备用 action: {self._alt_action}（主 action {self._action_name} 不可用）"
            )
            self._client = self._alt_client
            self._action_name = self._alt_action
            self._alt_client = None

    def _set_path(self, msg: Path, from_processed: bool) -> None:
        if not msg.poses:
            if from_processed:
                return
            self._path = None
            return
        if from_processed or not self._path_from_processed:
            self._path = msg
            self._path_from_processed = from_processed
        if not bool(self.get_parameter("require_start_command").value):
            self._start_follow()
        elif self._pending_start:
            self._start_follow()

    def _on_processed_path(self, msg: Path) -> None:
        self._set_path(msg, from_processed=True)

    def _on_fallback_path(self, msg: Path) -> None:
        self._set_path(msg, from_processed=False)

    def _on_start(self, msg: Bool) -> None:
        if msg.data:
            self._recovery_attempts = 0
            self._soft_abort_retries = 0
            self._pending_start = True
            self._start_follow()
        else:
            self._pending_start = False
            self._cancel_follow()

    def _on_stop(self, msg: Bool) -> None:
        if msg.data:
            self._stop_recovery()
            self._cancel_follow()

    def _controller_ready(self) -> bool:
        if self._client.server_is_ready():
            return True
        if self._alt_client is not None and self._alt_client.server_is_ready():
            self._client = self._alt_client
            self._action_name = self._alt_action
            self._alt_client = None
            return True
        return False

    def _reloc_ready(self) -> bool:
        if not bool(self.get_parameter("require_reloc_ok").value):
            return True
        return self._reloc_ok

    def _on_bringup_status(self, msg: String) -> None:
        prev = self._bringup_status
        self._bringup_status = msg.data.strip() or "PENDING"
        if self._pending_start and self._controller_ready() and prev != self._bringup_status:
            self.get_logger().info(
                f"controller 就绪 (status={self._bringup_status})，自动发送 FollowPath"
            )
            self._start_follow()

    def _stop_pre_align(self) -> None:
        self._pre_aligning = False
        self._prepared_path_pending = None
        self._pre_align_started_at = None

    def _stop_recovery(self) -> None:
        self._recovering = False
        self._recovery_phase = ""
        self._recovery_phase_end = None
        self._stop_pre_align()
        self._cmd_vel_pub.publish(Twist())

    def _schedule_soft_retry(self, reason: str) -> None:
        pause = float(self.get_parameter("soft_abort_retry_pause_sec").value)
        self._pending_start = True
        self._navigating = False
        self._status_pub.publish(String(data="MPPI_RETRY"))
        max_soft = int(self.get_parameter("soft_abort_retry_max").value)
        dist_part = (
            f"，距目标 {self._last_dist_to_goal:.1f}m"
            if self._last_dist_to_goal is not None
            else ""
        )
        self.get_logger().warn(
            f"{reason} → {pause:.1f}s 后软重试 FollowPath "
            f"({self._soft_abort_retries}/{max_soft}){dist_part}",
            throttle_duration_sec=3.0,
        )
        if hasattr(self, "_retry_timer") and self._retry_timer is not None:
            self._retry_timer.cancel()
        self._retry_timer = self.create_timer(pause, self._on_recovery_retry_once)

    def _should_escape_on_abort(self) -> bool:
        min_dist = float(self.get_parameter("escape_min_dist_to_goal_m").value)
        min_follow = float(self.get_parameter("escape_min_follow_sec").value)
        dist = self._last_dist_to_goal
        follow_sec = 0.0
        if self._goal_accept_mono is not None:
            follow_sec = time.monotonic() - self._goal_accept_mono
        if dist is None:
            return False
        if dist > min_dist:
            return False
        if follow_sec < min_follow:
            return False
        return True

    def _start_recovery(self) -> None:
        """仅在接近目标且已跟踪一段时间仍 ABORT 时后退+旋转脱困。"""
        if not self.get_parameter("enable_escape_recovery").value:
            self._pending_start = True
            return
        max_att = int(self.get_parameter("max_recovery_attempts").value)
        if self._recovery_attempts >= max_att:
            self.get_logger().error(
                f"脱困已尝试 {max_att} 次仍失败（局部 costmap 可能仍全障碍），请人工介入"
            )
            self._pending_start = False
            self._status_pub.publish(String(data="ESCAPE_FAILED"))
            return
        self._recovery_attempts += 1
        self._recovering = True
        self._recovery_phase = "backup"
        dur = float(self.get_parameter("backup_duration_sec").value)
        self._recovery_phase_end = self.get_clock().now() + rclpy.duration.Duration(
            seconds=dur
        )
        self._rotate_sign = 1.0 if (self._recovery_attempts % 2) else -1.0
        self._status_pub.publish(String(data="ESCAPE_BACKUP"))
        self.get_logger().warn(
            f"MPPI Optimizer 全碰撞 → 脱困后退 {dur:.1f}s "
            f"(第 {self._recovery_attempts}/{max_att} 次，非到达目标)"
        )

    def _finish_recovery(self) -> None:
        self._cmd_vel_pub.publish(Twist())
        self._recovering = False
        self._recovery_phase = ""
        self._recovery_phase_end = None
        pause = float(self.get_parameter("recovery_pause_sec").value)
        self._status_pub.publish(String(data="PATH_READY"))
        self.get_logger().info(f"脱困完成，{pause:.1f}s 后重试 FollowPath")
        self._pending_start = True
        if hasattr(self, "_retry_timer") and self._retry_timer is not None:
            self._retry_timer.cancel()
        self._retry_timer = self.create_timer(pause, self._on_recovery_retry_once)

    def _on_recovery_retry_once(self) -> None:
        if self._retry_timer is not None:
            self._retry_timer.cancel()
        if self._pending_start and not self._recovering:
            self._start_follow()

    def _on_recovery_timer(self) -> None:
        if self._pre_aligning:
            self._tick_pre_align()
            return
        if not self._recovering or self._recovery_phase_end is None:
            return
        now = self.get_clock().now()
        cmd = Twist()
        if self._recovery_phase == "backup":
            cmd.linear.x = -abs(float(self.get_parameter("backup_speed").value))
            if now >= self._recovery_phase_end:
                self._recovery_phase = "rotate"
                dur = float(self.get_parameter("rotate_duration_sec").value)
                self._recovery_phase_end = now + rclpy.duration.Duration(seconds=dur)
                self._status_pub.publish(String(data="ESCAPE_ROTATE"))
                self.get_logger().info(
                    f"脱困: 原地旋转 {dur:.1f}s (wz={'+' if self._rotate_sign > 0 else '-'})"
                )
        elif self._recovery_phase == "rotate":
            cmd.angular.z = self._rotate_sign * abs(
                float(self.get_parameter("rotate_speed").value)
            )
            if now >= self._recovery_phase_end:
                self._finish_recovery()
                return
        self._cmd_vel_pub.publish(cmd)

    def _on_timer(self) -> None:
        if self._recovering or self._pre_aligning:
            return
        if not self._reloc_ready():
            self._status_pub.publish(String(data="WAIT_RELOC"))
            return
        if (
            self._pending_start
            and self._path is not None
            and self._controller_ready()
            and not self._navigating
        ):
            self._start_follow()
        if not self._controller_ready():
            st = self._bringup_status
            if st in ("CONTROLLER_CONFIGURING", "CONTROLLER_ACTIVATING", "COSTMAP_CONFIGURING"):
                self._status_pub.publish(String(data="WAIT_CONTROLLER"))
            elif st in ("CONFIGURE_FAILED", "ACTIVATE_FAILED", "BRINGUP_FAILED"):
                self._status_pub.publish(String(data="CONTROLLER_FAILED"))
            elif st == "COSTMAP_ACTIVE":
                self._status_pub.publish(String(data="WAIT_CONTROLLER"))
            else:
                self._status_pub.publish(String(data="WAIT_CONTROLLER"))
            return
        if self._navigating and self._goal_handle is not None:
            status = self._goal_handle.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._status_pub.publish(String(data="GOAL_REACHED"))
                self._navigating = False
            elif status == GoalStatus.STATUS_ABORTED:
                self._status_pub.publish(String(data="ABORTED"))
                self._navigating = False
            elif status == GoalStatus.STATUS_CANCELED:
                self._status_pub.publish(String(data="CANCELED"))
                self._navigating = False
            else:
                self._status_pub.publish(String(data="MPPI_FOLLOWING"))
                if self._last_mppi_cmd_time is not None:
                    idle = (
                        self.get_clock().now() - self._last_mppi_cmd_time
                    ).nanoseconds * 1e-9
                    if idle > 3.0:
                        self.get_logger().warn(
                            f"FollowPath 运行中但 {idle:.0f}s 无 /cmd_vel："
                            "检查 /nav2_odom、costmap 是否 valid、MPPI 是否全碰撞",
                            throttle_duration_sec=8.0,
                        )
        elif self._path is not None:
            self._status_pub.publish(String(data="PATH_READY"))
        else:
            self._status_pub.publish(String(data="IDLE"))

    @staticmethod
    def _normalize_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    @classmethod
    def _angle_diff(cls, from_yaw: float, to_yaw: float) -> float:
        return cls._normalize_angle(to_yaw - from_yaw)

    def _lookup_robot_pose(self) -> tuple[float, float, float] | None:
        timeout = Duration(seconds=self._tf_timeout)
        frames = [self._base_frame]
        if self._base_frame_alt and self._base_frame_alt not in frames:
            frames.append(self._base_frame_alt)
        for base in frames:
            for stamp in (Time(), None):
                try:
                    if stamp is None:
                        tf = self._tf_buffer.lookup_transform(
                            self._map_frame,
                            base,
                            Time(),
                            timeout=timeout,
                        )
                    else:
                        tf = self._tf_buffer.lookup_transform(
                            self._map_frame,
                            base,
                            stamp,
                            timeout=timeout,
                        )
                except TransformException:
                    continue
                t = tf.transform.translation
                q = tf.transform.rotation
                yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
                return float(t.x), float(t.y), yaw
        return None

    def _path_heading_at_robot(self, path: Path) -> float:
        robot = self._lookup_robot_pose()
        if robot is None or not path.poses:
            ps = path.poses[0]
            o = ps.pose.orientation
            return self._yaw_from_quaternion(o.x, o.y, o.z, o.w)
        rx, ry, _ = robot
        best_i = 0
        best_d = float("inf")
        for i, ps in enumerate(path.poses):
            p = ps.pose.position
            d = (p.x - rx) ** 2 + (p.y - ry) ** 2
            if d < best_d:
                best_d = d
                best_i = i
        o = path.poses[best_i].pose.orientation
        return self._yaw_from_quaternion(o.x, o.y, o.z, o.w)

    def _maybe_start_pre_align(self, prepared_path: Path) -> bool:
        if not bool(self.get_parameter("enable_pre_align").value):
            return False
        robot = self._lookup_robot_pose()
        if robot is None:
            self.get_logger().warn(
                "无法查询 map->base TF，跳过起始对齐",
                throttle_duration_sec=5.0,
            )
            return False
        _, _, yaw = robot
        target = self._path_heading_at_robot(prepared_path)
        err = self._angle_diff(yaw, target)
        thr = math.radians(float(self.get_parameter("pre_align_threshold_deg").value))
        if abs(err) <= thr:
            return False
        self._pre_aligning = True
        self._pre_align_target_yaw = target
        self._prepared_path_pending = prepared_path
        self._pre_align_started_at = self.get_clock().now()
        self._status_pub.publish(String(data="PRE_ALIGN"))
        self.get_logger().info(
            f"起始朝向偏差 {math.degrees(err):.1f}°（阈值 {math.degrees(thr):.0f}°），"
            f"先原地旋转再启动 MPPI"
        )
        return True

    def _tick_pre_align(self) -> None:
        if self._pre_align_started_at is not None:
            timeout = float(self.get_parameter("pre_align_timeout_sec").value)
            elapsed = (self.get_clock().now() - self._pre_align_started_at).nanoseconds * 1e-9
            if elapsed >= timeout:
                prepared = self._prepared_path_pending
                self._stop_pre_align()
                self.get_logger().warn(
                    f"起始对齐超时 {timeout:.0f}s，跳过旋转直接启动 FollowPath"
                )
                if prepared is not None:
                    self._send_follow_path(prepared)
                return
        robot = self._lookup_robot_pose()
        if robot is None:
            self.get_logger().warn(
                f"PRE_ALIGN 无 TF {self._map_frame}->{self._base_frame}，等待…",
                throttle_duration_sec=4.0,
            )
            return
        _, _, yaw = robot
        err = self._angle_diff(yaw, self._pre_align_target_yaw)
        tol = math.radians(float(self.get_parameter("pre_align_tolerance_deg").value))
        if abs(err) <= tol:
            self._pre_aligning = False
            self._cmd_vel_pub.publish(Twist())
            prepared = self._prepared_path_pending
            self._prepared_path_pending = None
            if prepared is not None:
                self.get_logger().info(
                    f"起始对齐完成（残差 {math.degrees(err):.1f}°），启动 FollowPath"
                )
                self._send_follow_path(prepared)
            return
        wz = float(self.get_parameter("pre_align_wz").value)
        cmd = Twist()
        cmd.angular.z = wz if err > 0.0 else -wz
        self._cmd_vel_pub.publish(cmd)

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _set_yaw(pose, yaw: float) -> None:
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(yaw * 0.5)
        pose.orientation.w = math.cos(yaw * 0.5)

    def _yaw_lookahead(self, poses, index: int, lookahead_m: float) -> float:
        """沿路径向前看 lookahead_m，得到局部目标行进方向（比切线/平滑更稳）。"""
        n = len(poses)
        p0 = poses[index].pose.position
        acc = 0.0
        j = index
        while j + 1 < n:
            p1 = poses[j + 1].pose.position
            seg = math.hypot(p1.x - poses[j].pose.position.x, p1.y - poses[j].pose.position.y)
            acc += seg
            j += 1
            if acc >= lookahead_m:
                pt = poses[j].pose.position
                return math.atan2(pt.y - p0.y, pt.x - p0.x)
        if index + 1 < n:
            pt = poses[-1].pose.position
            return math.atan2(pt.y - p0.y, pt.x - p0.x)
        return 0.0

    def _prepare_path_for_mppi(self, path: Path) -> Path:
        """中间点：前视路径方向；终点：保留 GUI/dong_path 目标 yaw。"""
        if not path.poses:
            return path

        out = copy.deepcopy(path)
        poses = out.poses
        n = len(poses)
        lookahead_m = float(self.get_parameter("yaw_lookahead_m").value)

        if not self.get_parameter("preserve_global_path_yaw").value:
            for i in range(n):
                if i < n - 1:
                    self._set_yaw(poses[i].pose, self._yaw_lookahead(poses, i, lookahead_m))
            return out

        yaws: list[float] = []
        for ps in poses:
            o = ps.pose.orientation
            if abs(o.w) < 1e-6 and abs(o.z) < 1e-6:
                yaws.append(float("nan"))
            else:
                yaws.append(self._yaw_from_quaternion(o.x, o.y, o.z, o.w))

        goal_yaw = yaws[-1] if not math.isnan(yaws[-1]) else self._yaw_lookahead(
            poses, n - 1, lookahead_m
        )

        for i in range(n - 1):
            yaws[i] = self._yaw_lookahead(poses, i, lookahead_m)
        yaws[-1] = goal_yaw

        if self.get_parameter("smooth_path_yaw").value and n >= 3:
            win = max(1, int(self.get_parameter("yaw_smooth_window").value))
            half = win // 2
            smoothed: list[float] = []
            for i in range(n):
                z_sum = 0.0
                w_sum = 0.0
                for j in range(max(0, i - half), min(n, i + half + 1)):
                    z_sum += math.sin(yaws[j])
                    w_sum += math.cos(yaws[j])
                smoothed.append(math.atan2(z_sum, w_sum))
            yaws = smoothed
            yaws[-1] = goal_yaw

        spacing = float(self.get_parameter("downsample_min_spacing_m").value)
        if spacing > 0.0 and n > 2:
            kept = [poses[0]]
            kept_yaws = [yaws[0]]
            for i in range(1, n - 1):
                p = poses[i].pose.position
                last = kept[-1].pose.position
                if math.hypot(p.x - last.x, p.y - last.y) >= spacing:
                    kept.append(poses[i])
                    kept_yaws.append(yaws[i])
            kept.append(poses[-1])
            kept_yaws.append(goal_yaw)
            poses = kept
            yaws = kept_yaws
            out.poses = poses

        for ps, yaw in zip(poses, yaws):
            self._set_yaw(ps.pose, yaw)

        self.get_logger().info(
            f"路径朝向→MPPI: {len(poses)} 点, 前视={lookahead_m:.1f}m, "
            f"终点 yaw={math.degrees(goal_yaw):.1f}°",
            throttle_duration_sec=3.0,
        )
        return out

    def _on_cmd_vel(self, msg: Twist) -> None:
        if abs(msg.linear.x) + abs(msg.angular.z) < 1e-4:
            return
        self._last_mppi_cmd_time = self.get_clock().now()
        self.get_logger().info(
            f"/cmd_vel vx={msg.linear.x:.3f} wz={msg.angular.z:.3f}",
            throttle_duration_sec=2.0,
        )

    def _send_follow_path(self, prepared_path: Path) -> None:
        goal = FollowPath.Goal()
        goal.path = prepared_path
        stamp = self.get_clock().now().to_msg()
        goal.path.header.stamp = stamp
        for pose in goal.path.poses:
            pose.header.stamp = stamp
            if not pose.header.frame_id:
                pose.header.frame_id = goal.path.header.frame_id or "map"
        goal.controller_id = self.get_parameter("controller_id").value
        goal.goal_checker_id = self.get_parameter("goal_checker_id").value

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        send_future.add_done_callback(self._on_goal_response)
        n = len(prepared_path.poses)
        frame = prepared_path.header.frame_id or "map"
        self.get_logger().info(f"FollowPath goal sent: {n} poses, frame={frame}")

    def _start_follow(self) -> None:
        if self._recovering or self._pre_aligning:
            return
        if not self._reloc_ready():
            self.get_logger().warn(
                "未重定位，MPPI 需要 map->base_link TF，请完成 ICP 后再开始导航",
                throttle_duration_sec=6.0,
            )
            self._pending_start = True
            self._status_pub.publish(String(data="WAIT_RELOC"))
            return
        if self._path is None or not self._path.poses:
            self.get_logger().warn(
                "start_navigation ignored: 无 /global_path_processed 或 /planned_path",
                throttle_duration_sec=5.0,
            )
            return
        if not self._controller_ready():
            self.get_logger().warn(
                f"FollowPath 未就绪 (status={self._bringup_status}, "
                f"action={self._action_name})，约 40–60s 后自动重试",
                throttle_duration_sec=8.0,
            )
            self._pending_start = True
            self._status_pub.publish(String(data="WAIT_CONTROLLER"))
            return
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                f"FollowPath action {self._action_name} 无响应（controller_server 未 active？）",
                throttle_duration_sec=8.0,
            )
            self._pending_start = True
            self._status_pub.publish(String(data="WAIT_CONTROLLER"))
            return
        if self._goal_handle is not None:
            self._cancel_follow()

        prepared = self._prepare_path_for_mppi(self._path)
        if self._maybe_start_pre_align(prepared):
            return
        self._send_follow_path(prepared)

    def _on_goal_response(self, future) -> None:
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error(
                "FollowPath goal rejected（controller_id/goal_checker 或路径 frame 无效？）"
            )
            self._navigating = False
            self._pending_start = True
            self._status_pub.publish(String(data="REJECTED"))
            return
        self._navigating = True
        self._pending_start = False
        self._last_mppi_cmd_time = None
        self._goal_accept_mono = time.monotonic()
        self._last_dist_to_goal = None
        self._status_pub.publish(String(data="MPPI_FOLLOWING"))
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self._last_dist_to_goal = float(fb.distance_to_goal)
        if fb.speed > 0.08:
            self._soft_abort_retries = max(0, self._soft_abort_retries - 1)
        self.get_logger().info(
            f"MPPI tracking dist={fb.distance_to_goal:.2f}m speed={fb.speed:.2f}m/s",
            throttle_duration_sec=2.0,
        )

    def _on_result(self, future) -> None:
        result = future.result()
        status = result.status
        self._navigating = False
        self._goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("FollowPath succeeded")
            self._recovery_attempts = 0
            self._soft_abort_retries = 0
            self._status_pub.publish(String(data="GOAL_REACHED"))
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("FollowPath canceled")
            self._status_pub.publish(String(data="CANCELED"))
        else:
            dist_s = (
                f"{self._last_dist_to_goal:.1f}m"
                if self._last_dist_to_goal is not None
                else "?"
            )
            self.get_logger().warn(
                f"FollowPath ABORT(status={status})，距目标 {dist_s}（Optimizer 全碰撞/参数过严）"
            )
            self._status_pub.publish(String(data="ABORTED"))
            if not self.get_parameter("retry_on_abort").value:
                return
            max_soft = int(self.get_parameter("soft_abort_retry_max").value)
            if not self._should_escape_on_abort():
                self._soft_abort_retries += 1
                if self._soft_abort_retries <= max_soft:
                    self._schedule_soft_retry("距目标仍较远或刚启动即失败")
                    return
                self.get_logger().error(
                    f"软重试 {max_soft} 次仍 ABORT，请检查 MPPI costmap 是否过满"
                )
                self._status_pub.publish(String(data="MPPI_FAILED"))
                self._pending_start = False
                return
            self._start_recovery()

    def _cancel_follow(self) -> None:
        self._stop_recovery()
        self._navigating = False
        if self._goal_handle is None:
            self._status_pub.publish(String(data="IDLE"))
            return
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._on_cancel_done)

    def _on_cancel_done(self, future) -> None:
        _ = future.result()
        self._goal_handle = None
        self._status_pub.publish(String(data="CANCELED"))
        self.get_logger().info("FollowPath canceled")


def main() -> None:
    rclpy.init()
    node = MppiPathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
