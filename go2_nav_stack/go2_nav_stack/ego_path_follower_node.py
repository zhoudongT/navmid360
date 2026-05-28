#!/usr/bin/env python3
"""将 /planned_path 发给 EGO：整段(global_path) 或逐点(per_waypoint/lookahead)。"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
import tf2_ros
from tf2_ros import TransformException


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    h = yaw * 0.5
    return (0.0, 0.0, math.sin(h), math.cos(h))


def _normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class EgoPathFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("ego_path_follower")
        self.declare_parameter("planned_path_topic", "/planned_path")
        self.declare_parameter("ego_global_path_topic", "/ego_global_path")
        self.declare_parameter("start_navigation_topic", "/start_navigation")
        self.declare_parameter("stop_navigation_topic", "/stop_navigation")
        self.declare_parameter("ego_goal_topic", "/move_base_simple/goal")
        self.declare_parameter("ego_trigger_topic", "/traj_start_trigger")
        self.declare_parameter("require_start_command", True)
        # global_path：整段路径 -> /ego_global_path；per_waypoint/lookahead：单点 goal
        self.declare_parameter("follow_mode", "global_path")
        self.declare_parameter("lookahead_m", 1.5)
        self.declare_parameter("max_corner_yaw_deg", 55.0)
        self.declare_parameter("goal_z", 0.27)
        self.declare_parameter("preserve_path_z", True)
        self.declare_parameter("arrival_xy_m", 0.15)
        self.declare_parameter("arrival_z_m", 0.20)
        self.declare_parameter("yaw_arrival_deg", 35.0)
        self.declare_parameter("require_yaw_arrival", False)
        self.declare_parameter("stuck_advance_sec", 5.0)
        self.declare_parameter("stuck_advance_max_dist_m", 0.85)
        self.declare_parameter("stuck_advance_force_dwell_sec", 8.0)
        self.declare_parameter("follow_tick_hz", 5.0)
        self.declare_parameter("goal_heartbeat_sec", 3.0)
        self.declare_parameter("global_path_republish_sec", 0.0)
        self.declare_parameter("status_topic", "/local_planner/status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 0.2)

        self._path: Path | None = None
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._robot_frame = str(self.get_parameter("robot_frame").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self._buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._listener = tf2_ros.TransformListener(self._buffer, self)
        self._navigating = False
        self._goal_z = float(self.get_parameter("goal_z").value)
        self._preserve_path_z = bool(self.get_parameter("preserve_path_z").value)
        self._arrival = float(self.get_parameter("arrival_xy_m").value)
        self._arrival_z = float(self.get_parameter("arrival_z_m").value)
        self._yaw_arrival = math.radians(
            float(self.get_parameter("yaw_arrival_deg").value)
        )
        self._require_yaw = bool(self.get_parameter("require_yaw_arrival").value)
        self._stuck_sec = float(self.get_parameter("stuck_advance_sec").value)
        self._stuck_max_dist = float(
            self.get_parameter("stuck_advance_max_dist_m").value
        )
        self._stuck_force_dwell = float(
            self.get_parameter("stuck_advance_force_dwell_sec").value
        )
        self._follow_mode = str(self.get_parameter("follow_mode").value).strip()
        self._global_path_mode = self._follow_mode in (
            "global_path",
            "full_path",
            "path",
        )
        self._per_waypoint = self._follow_mode in ("per_waypoint", "sequential", "node")
        self._lookahead_m = float(self.get_parameter("lookahead_m").value)
        self._max_corner_yaw = math.radians(
            float(self.get_parameter("max_corner_yaw_deg").value)
        )
        self._heartbeat = float(self.get_parameter("goal_heartbeat_sec").value)
        self._current_wp = 0
        self._wp_enter_t = 0.0
        self._last_published_wp = -1
        self._last_publish_t = 0.0
        self._nav_start_t = 0.0
        self._global_repub_sec = max(
            0.0, float(self.get_parameter("global_path_republish_sec").value)
        )

        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("ego_goal_topic").value), 10
        )
        self._ego_path_pub = self.create_publisher(
            Path,
            str(self.get_parameter("ego_global_path_topic").value),
            path_qos,
        )
        self._trigger_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("ego_trigger_topic").value), 10
        )

        self.create_subscription(
            Path,
            str(self.get_parameter("planned_path_topic").value),
            self._on_path,
            path_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("start_navigation_topic").value),
            self._on_start,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("stop_navigation_topic").value),
            self._on_stop,
            10,
        )
        hz = max(1.0, float(self.get_parameter("follow_tick_hz").value))
        self.create_timer(1.0 / hz, self._tick)
        if self._global_path_mode:
            mode_s = "整段路径"
            out_s = str(self.get_parameter("ego_global_path_topic").value)
        elif self._per_waypoint:
            mode_s = "逐点"
            out_s = "/move_base_simple/goal"
        else:
            mode_s = f"前视{self._lookahead_m}m"
            out_s = "/move_base_simple/goal"
        self.get_logger().info(
            f"ego_path_follower: {mode_s} 跟踪 /planned_path -> {out_s}"
        )

    def _segment_yaw_rad(self, seg_start: int) -> float:
        assert self._path is not None
        poses = self._path.poses
        p0 = poses[seg_start].pose.position
        p1 = poses[seg_start + 1].pose.position
        return math.atan2(p1.y - p0.y, p1.x - p0.x)

    def _turn_at_vertex(self, vtx: int) -> float:
        assert self._path is not None
        n = len(self._path.poses)
        if vtx <= 0 or vtx >= n - 1:
            return 0.0
        y_in = self._segment_yaw_rad(vtx - 1)
        y_out = self._segment_yaw_rad(vtx)
        return abs(_normalize_angle(y_out - y_in))

    def _analyze_planned_path(self) -> None:
        assert self._path is not None
        n = len(self._path.poses)
        sharp: list[tuple[int, float]] = []
        for i in range(1, n - 1):
            turn = self._turn_at_vertex(i)
            if turn >= self._max_corner_yaw:
                sharp.append((i, math.degrees(turn)))
        if sharp:
            preview = ", ".join(f"wp{idx}={deg:.0f}°" for idx, deg in sharp[:6])
            extra = f" 等{len(sharp)}处" if len(sharp) > 6 else ""
            self.get_logger().warn(
                f"全局路径大转角 {len(sharp)} 处（>{math.degrees(self._max_corner_yaw):.0f}°）: "
                f"{preview}{extra}"
            )
        else:
            self.get_logger().info(
                f"全局路径 {n} 个节点，转角检查通过（阈值 "
                f"{math.degrees(self._max_corner_yaw):.0f}°）"
            )

    def _goal_index(self) -> int:
        if self._per_waypoint:
            return self._current_wp
        return self._lookahead_goal_index()

    def _on_path(self, msg: Path) -> None:
        if msg.poses:
            self._path = msg
            self._analyze_planned_path()
            self._status_pub.publish(String(data="PATH_READY"))
            if not self.get_parameter("require_start_command").value:
                self._begin_navigation()

    def _on_start(self, msg: Bool) -> None:
        if msg.data:
            self._begin_navigation()

    def _on_stop(self, msg: Bool) -> None:
        if msg.data:
            self._navigating = False
            self._status_pub.publish(String(data="CLEAR"))

    def _begin_navigation(self) -> None:
        if self._path is None or not self._path.poses:
            self.get_logger().warn("start_navigation: 无 /planned_path")
            return
        self._navigating = True
        self._nav_start_t = time.monotonic()
        self._current_wp = self._find_start_index()
        self._wp_enter_t = time.monotonic()
        self._last_published_wp = -1
        trig = PoseStamped()
        trig.header.stamp = self.get_clock().now().to_msg()
        trig.header.frame_id = self._path.header.frame_id or "map"
        self._trigger_pub.publish(trig)
        if self._global_path_mode:
            self._publish_global_path(force=True)
            self._publish_terminal_goal()
        else:
            self._publish_waypoint(self._goal_index(), force=True)
        self._status_pub.publish(String(data="EGO_FOLLOWING"))

    def _publish_global_path(self, *, force: bool = False) -> None:
        if self._path is None or not self._path.poses:
            return
        now = time.monotonic()
        if not force and now - self._last_publish_t < self._heartbeat:
            return
        out = Path()
        out.header = self._path.header
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = out.header.frame_id or "map"
        out.poses = list(self._path.poses)
        if not self._preserve_path_z:
            for ps in out.poses:
                ps.pose.position.z = self._goal_z
        self._ego_path_pub.publish(out)
        self._last_publish_t = now
        n = len(out.poses)
        end = out.poses[-1].pose.position
        self.get_logger().info(
            f"[path_follower] 整段路径 -> EGO: {n} 点 "
            f"终点=({end.x:.2f},{end.y:.2f}) start_wp={self._current_wp}",
        )

    def _publish_terminal_goal(self) -> None:
        """供 ego_cmd_bridge 无 pos_cmd 时朝终点 fallback。"""
        if self._path is None or not self._path.poses:
            return
        idx = len(self._path.poses) - 1
        ps = self._path.poses[idx]
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._path.header.frame_id or "map"
        goal.pose = ps.pose
        if not self._preserve_path_z:
            goal.pose.position.z = self._goal_z
        yaw = self._waypoint_yaw_rad(idx)
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        self._goal_pub.publish(goal)

    def _robot_xyz(self) -> tuple[float, float, float] | None:
        try:
            tf = self._buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        return float(t.x), float(t.y), float(t.z)

    def _goal_reached(
        self, x: float, y: float, z: float, *, idx: int | None = None
    ) -> bool:
        rxyz = self._robot_xyz()
        if rxyz is None:
            return False
        rx, ry, rz = rxyz
        xy = math.hypot(x - rx, y - ry)
        dz = abs(z - rz)
        if xy > self._arrival or dz > self._arrival_z:
            return False
        if not self._require_yaw:
            return True
        ryaw = self._robot_yaw()
        if ryaw is None:
            return True
        wyaw = (
            self._waypoint_yaw_rad(idx)
            if idx is not None
            else math.atan2(y - ry, x - rx)
        )
        return abs(_normalize_angle(wyaw - ryaw)) <= self._yaw_arrival

    def _dist_to_robot(self, x: float, y: float) -> float | None:
        try:
            tf = self._buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except TransformException:
            return None
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        return math.hypot(x - rx, y - ry)

    def _robot_pose(self) -> tuple[float, float, float] | None:
        try:
            tf = self._buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        yaw = _yaw_from_quat(r.x, r.y, r.z, r.w)
        return float(t.x), float(t.y), yaw

    def _robot_yaw(self) -> float | None:
        pose = self._robot_pose()
        return pose[2] if pose is not None else None

    def _find_start_index(self) -> int:
        assert self._path is not None
        poses = self._path.poses
        n = len(poses)
        if n <= 1:
            return 0
        best_i = 0
        best_d = 1e9
        search_end = max(2, int(n * 0.5))
        for i in range(search_end):
            p = poses[i].pose.position
            d = self._dist_to_robot(p.x, p.y)
            if d is not None and d < best_d:
                best_d = d
                best_i = i
        return best_i

    def _waypoint_yaw_rad(self, idx: int) -> float:
        assert self._path is not None
        poses = self._path.poses
        n = len(poses)
        idx = min(max(0, idx), n - 1)
        o = poses[idx].pose.orientation
        qn = math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w)
        if qn > 0.1:
            return _yaw_from_quat(o.x, o.y, o.z, o.w)
        if idx < n - 1:
            p0 = poses[idx].pose.position
            p1 = poses[idx + 1].pose.position
        elif idx > 0:
            p0 = poses[idx - 1].pose.position
            p1 = poses[idx].pose.position
        else:
            return 0.0
        return math.atan2(p1.y - p0.y, p1.x - p0.x)

    def _advance_progress_along_route(self) -> int:
        """沿路线推进：不必踩中每个节点，经过或更接近下一点即前移。"""
        assert self._path is not None
        poses = self._path.poses
        n = len(poses)
        advanced = 0
        pass_radius = max(self._arrival, self._stuck_max_dist * 0.85)
        while self._current_wp < n - 1:
            p = poses[self._current_wp].pose.position
            d_cur = self._dist_to_robot(p.x, p.y)
            if d_cur is None:
                break
            if d_cur < pass_radius:
                self._current_wp += 1
                advanced += 1
                continue
            pn = poses[self._current_wp + 1].pose.position
            d_next = self._dist_to_robot(pn.x, pn.y)
            if d_next is not None and d_next + 0.08 < d_cur:
                self._current_wp += 1
                advanced += 1
                continue
            break
        return advanced

    def _lookahead_goal_index(self) -> int:
        assert self._path is not None
        poses = self._path.poses
        n = len(poses)
        if n <= 1:
            return 0
        start_i = min(max(0, self._current_wp), n - 1)
        acc = 0.0
        target_i = start_i
        for j in range(start_i, n - 1):
            p0 = poses[j].pose.position
            p1 = poses[j + 1].pose.position
            seg = math.hypot(p1.x - p0.x, p1.y - p0.y)
            acc += seg
            next_i = j + 1
            turn = self._turn_at_vertex(next_i)
            target_i = next_i
            if turn >= self._max_corner_yaw:
                break
            if acc >= self._lookahead_m:
                break
        if target_i >= n - 1 and self._current_wp < n - 2:
            target_i = n - 1
        target_i = min(target_i, n - 1)
        if target_i == start_i and start_i < n - 1:
            target_i = start_i + 1
        return target_i

    def _publish_waypoint(self, index: int, *, force: bool = False) -> None:
        if self._path is None or not self._path.poses:
            return
        n = len(self._path.poses)
        idx = min(max(0, index), n - 1)
        now = time.monotonic()
        if (
            not force
            and idx == self._last_published_wp
            and now - self._last_publish_t < self._heartbeat
        ):
            return

        ps = self._path.poses[idx]
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._path.header.frame_id or "map"
        goal.pose = ps.pose
        if not self._preserve_path_z:
            goal.pose.position.z = self._goal_z
        yaw = self._waypoint_yaw_rad(idx)
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        if idx != self._last_published_wp:
            self._wp_enter_t = now
        self._goal_pub.publish(goal)
        self._last_published_wp = idx
        self._last_publish_t = now
        dist = self._dist_to_robot(
            goal.pose.position.x, goal.pose.position.y
        )
        rpose = self._robot_pose()
        dwell = now - self._wp_enter_t
        reached = (
            dist is not None
            and dist <= self._arrival
            and self._waypoint_reached(idx)
        )
        stuck_zone = dist is not None and dist < self._stuck_max_dist
        force_dwell = dwell >= self._stuck_force_dwell and not reached
        stuck_ready = (stuck_zone and dwell >= self._stuck_sec) or force_dwell
        mode_s = "逐点" if self._per_waypoint else "前视"
        robot_s = " robot=?"
        yaw_err_deg = 0.0
        if rpose is not None:
            rx, ry, ryaw = rpose
            yaw_err_deg = math.degrees(_normalize_angle(yaw - ryaw))
            robot_s = (
                f" robot=({rx:.2f},{ry:.2f}) yaw={math.degrees(ryaw):.0f}°"
                f" err={yaw_err_deg:.0f}°"
            )
        self.get_logger().info(
            f"[path_follower] {mode_s} current_wp={self._current_wp} "
            f"goal_wp={idx}/{n - 1} "
            f"goal_xy=({goal.pose.position.x:.2f},{goal.pose.position.y:.2f}) "
            f"path_yaw={math.degrees(yaw):.0f}°{robot_s} "
            f"dist={dist if dist is not None else -1:.2f}m "
            f"arrival<{self._arrival:.2f}m "
            f"stuck_if_dist<{self._stuck_max_dist:.2f}m&dwell>{self._stuck_sec:.0f}s "
            f"dwell={dwell:.1f}s stuck_zone={stuck_zone} force_dwell={force_dwell} "
            f"stuck_ready={stuck_ready} "
            f"reached={reached} force={force}",
            throttle_duration_sec=0.0,
        )

    def _waypoint_reached(self, idx: int) -> bool:
        assert self._path is not None
        p = self._path.poses[idx].pose.position
        return self._goal_reached(p.x, p.y, p.z, idx=idx)

    def _tick(self) -> None:
        if not self._navigating or self._path is None or not self._path.poses:
            return

        n = len(self._path.poses)
        now = time.monotonic()

        if self._global_path_mode:
            self._advance_progress_along_route()
            goal_i = self._lookahead_goal_index()
            end = self._path.poses[n - 1].pose.position
            if self._goal_reached(end.x, end.y, end.z, idx=n - 1):
                self._finish_navigation()
                return
            # 沿路径前视更新 /move_base_simple/goal，避免全程只盯终点导致接近时乱走/倒车
            if goal_i != self._last_published_wp or now - self._last_publish_t >= self._heartbeat:
                self._publish_waypoint(goal_i, force=True)
            # 仅导航开始后短时间内重发整段路径，供 planner respawn 恢复
            if (
                self._global_repub_sec > 0.0
                and now - self._nav_start_t < self._global_repub_sec
                and now - self._last_publish_t >= self._heartbeat
            ):
                self._publish_global_path(force=True)
            return

        if self._per_waypoint:
            goal_i = self._current_wp
            if self._current_wp >= n - 1 and self._waypoint_reached(n - 1):
                self._finish_navigation()
                return
            if self._waypoint_reached(self._current_wp):
                if self._current_wp >= n - 1:
                    self._finish_navigation()
                    return
                self._current_wp += 1
                self._publish_waypoint(self._current_wp, force=True)
                return
            p = self._path.poses[self._current_wp].pose.position
            dist = self._dist_to_robot(p.x, p.y)
            dwell = now - self._wp_enter_t
            stuck_near = dist is not None and dist < self._stuck_max_dist
            stuck_time = stuck_near and dwell >= self._stuck_sec
            stuck_force = dwell >= self._stuck_force_dwell
            if (
                dist is not None
                and (stuck_time or stuck_force)
                and self._current_wp < n - 1
            ):
                reason = (
                    f"dwell>{self._stuck_force_dwell:.0f}s"
                    if stuck_force and not stuck_time
                    else f"dist<{self._stuck_max_dist:.2f}m&dwell>{self._stuck_sec:.0f}s"
                )
                self.get_logger().warn(
                    f"[path_follower] 超时前进 current_wp={self._current_wp} "
                    f"dist={dist:.2f}m dwell={dwell:.1f}s ({reason}) "
                    f"-> wp {self._current_wp + 1}"
                )
                self._current_wp += 1
                self._publish_waypoint(self._current_wp, force=True)
                return
        else:
            self._advance_progress_along_route()
            goal_i = self._lookahead_goal_index()
            if self._current_wp >= n - 1:
                end = self._path.poses[n - 1].pose.position
                if self._goal_reached(end.x, end.y, end.z, idx=n - 1):
                    self._finish_navigation()
                    return
            if goal_i != self._last_published_wp:
                self._publish_waypoint(goal_i, force=True)
                return

        if now - self._last_publish_t >= self._heartbeat:
            self._publish_waypoint(goal_i, force=True)

    def _finish_navigation(self) -> None:
        self._status_pub.publish(String(data="GOAL_REACHED"))
        self._navigating = False
        self.get_logger().info("[path_follower] 全局路径导航完成 GOAL_REACHED")


def main() -> None:
    rclpy.init()
    node = EgoPathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
