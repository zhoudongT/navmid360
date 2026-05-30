#!/usr/bin/env python3
"""沿 /planned_path 向前视发布 FAR /goal_point，使 FAR 贴近 OctoMap 全局路线。"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
import tf2_ros
from tf2_ros import TransformException


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class FarPathFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("far_path_follower")
        self.declare_parameter("planned_path_topic", "/planned_path")
        self.declare_parameter("far_goal_topic", "/goal_point")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 0.25)
        self.declare_parameter("lookahead_m", 2.0)
        self.declare_parameter("max_corner_yaw_deg", 70.0)
        self.declare_parameter("goal_z", 0.27)
        self.declare_parameter("preserve_path_z", True)
        self.declare_parameter("arrival_xy_m", 0.40)
        self.declare_parameter("follow_tick_hz", 5.0)
        self.declare_parameter("goal_republish_sec", 0.8)
        self.declare_parameter("require_start_command", True)
        self.declare_parameter("align_goal_z_to_robot", True)
        self.declare_parameter("graph_warmup_sec", 2.0)
        self.declare_parameter("require_relocalization", True)
        self.declare_parameter("waypoint_timeout_sec", 12.0)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._robot_frame = str(self.get_parameter("robot_frame").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self._lookahead_m = float(self.get_parameter("lookahead_m").value)
        self._max_corner_yaw = math.radians(
            float(self.get_parameter("max_corner_yaw_deg").value)
        )
        self._goal_z = float(self.get_parameter("goal_z").value)
        self._preserve_path_z = bool(self.get_parameter("preserve_path_z").value)
        self._arrival = float(self.get_parameter("arrival_xy_m").value)
        self._heartbeat = float(self.get_parameter("goal_republish_sec").value)
        self._require_start = bool(self.get_parameter("require_start_command").value)
        self._align_goal_z = bool(self.get_parameter("align_goal_z_to_robot").value)
        self._graph_warmup = float(self.get_parameter("graph_warmup_sec").value)
        self._require_reloc = bool(self.get_parameter("require_relocalization").value)
        self._wp_timeout = float(self.get_parameter("waypoint_timeout_sec").value)

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._goal_pub = self.create_publisher(
            PointStamped, str(self.get_parameter("far_goal_topic").value), 10
        )
        self._update_pub = self.create_publisher(Bool, "/update_visibility_graph", 5)

        self._buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._listener = tf2_ros.TransformListener(self._buffer, self)

        self._path: Path | None = None
        self._navigating = False
        self._current_wp = 0
        self._last_published_wp = -1
        self._last_publish_t = 0.0
        self._nav_ready_at = 0.0
        self._reloc_ok = not self._require_reloc
        self._first_goal_sent_at = 0.0
        self._saw_waypoint = False

        self.create_subscription(
            Path,
            str(self.get_parameter("planned_path_topic").value),
            self._on_path,
            path_qos,
        )
        self.create_subscription(Bool, "/start_navigation", self._on_start, 10)
        self.create_subscription(Bool, "/stop_navigation", self._on_stop, 10)
        self.create_subscription(Bool, "/relocalization/ok", self._on_reloc_ok, 10)
        self.create_subscription(PointStamped, "/way_point", self._on_waypoint, 10)

        hz = float(self.get_parameter("follow_tick_hz").value)
        self.create_timer(1.0 / max(hz, 2.0), self._tick)
        self.get_logger().info(
            f"far_path_follower: /planned_path 前视 {self._lookahead_m:.1f}m -> "
            f"{self.get_parameter('far_goal_topic').value}"
        )

    def _on_path(self, msg: Path) -> None:
        self._path = msg
        if self._navigating:
            self._current_wp = self._find_start_index()

    def _on_reloc_ok(self, msg: Bool) -> None:
        self._reloc_ok = bool(msg.data)

    def _on_waypoint(self, _msg: PointStamped) -> None:
        self._saw_waypoint = True

    def _on_start(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self._require_reloc and not self._reloc_ok:
            self.get_logger().error(
                "start_navigation 拒绝：未完成重定位。请在 Qt「当前姿态」点选位置，"
                "或重建 scan_context_db 后重启"
            )
            return
        if self._require_start and (self._path is None or not self._path.poses):
            self.get_logger().warn("start_navigation: 无 /planned_path")
            return
        self._navigating = True
        self._current_wp = self._find_start_index()
        self._last_published_wp = -1
        self._nav_ready_at = time.monotonic() + max(0.0, self._graph_warmup)
        self._first_goal_sent_at = 0.0
        self._saw_waypoint = False
        self.get_logger().info(
            f"FAR 沿全局路径导航: {len(self._path.poses)} 点, start_wp={self._current_wp}, "
            f"等待 V-Graph {self._graph_warmup:.0f}s 后发 /goal_point"
        )

    def _on_stop(self, msg: Bool) -> None:
        if msg.data:
            self._navigating = False

    def _robot_xy(self) -> tuple[float, float] | None:
        xyz = self._robot_xyz()
        return None if xyz is None else (xyz[0], xyz[1])

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

    def _dist_to_robot(self, x: float, y: float) -> float | None:
        r = self._robot_xy()
        if r is None:
            return None
        return math.hypot(x - r[0], y - r[1])

    def _find_start_index(self) -> int:
        if self._path is None or not self._path.poses:
            return 0
        poses = self._path.poses
        n = len(poses)
        best_i, best_d = 0, 1e9
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
            p0, p1 = poses[idx].pose.position, poses[idx + 1].pose.position
        elif idx > 0:
            p0, p1 = poses[idx - 1].pose.position, poses[idx].pose.position
        else:
            return 0.0
        return math.atan2(p1.y - p0.y, p1.x - p0.x)

    def _turn_at_vertex(self, idx: int) -> float:
        assert self._path is not None
        poses = self._path.poses
        if idx <= 0 or idx >= len(poses) - 1:
            return 0.0
        y0 = self._waypoint_yaw_rad(idx - 1)
        y1 = self._waypoint_yaw_rad(idx)
        return abs(_normalize_angle(y1 - y0))

    def _advance_progress_along_route(self) -> None:
        assert self._path is not None
        poses = self._path.poses
        n = len(poses)
        pass_radius = max(self._arrival, 0.35)
        while self._current_wp < n - 1:
            p = poses[self._current_wp].pose.position
            d_cur = self._dist_to_robot(p.x, p.y)
            if d_cur is None:
                break
            if d_cur < pass_radius:
                self._current_wp += 1
                continue
            pn = poses[self._current_wp + 1].pose.position
            d_next = self._dist_to_robot(pn.x, pn.y)
            if d_next is not None and d_next + 0.08 < d_cur:
                self._current_wp += 1
                continue
            break

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
            acc += math.hypot(p1.x - p0.x, p1.y - p0.y)
            next_i = j + 1
            target_i = next_i
            if self._turn_at_vertex(next_i) >= self._max_corner_yaw:
                break
            if acc >= self._lookahead_m:
                break
        target_i = min(target_i, n - 1)
        if target_i == start_i and start_i < n - 1:
            target_i = start_i + 1
        return target_i

    def _publish_goal(self, index: int, *, force: bool = False) -> None:
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
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._path.header.frame_id or self._map_frame
        out.point = ps.pose.position
        if self._align_goal_z:
            r = self._robot_xyz()
            if r is not None:
                out.point.z = r[2]
        elif not self._preserve_path_z:
            out.point.z = self._goal_z
        self._goal_pub.publish(out)
        upd = Bool()
        upd.data = True
        self._update_pub.publish(upd)
        self._last_published_wp = idx
        self._last_publish_t = now
        if self._first_goal_sent_at <= 0.0:
            self._first_goal_sent_at = now
        self.get_logger().info(
            f"FAR /goal_point wp={idx}/{n - 1} "
            f"({out.point.x:.2f}, {out.point.y:.2f}, {out.point.z:.2f})",
            throttle_duration_sec=2.0,
        )

    def _tick(self) -> None:
        if not self._navigating or self._path is None or not self._path.poses:
            return
        if self._require_reloc and not self._reloc_ok:
            return
        if time.monotonic() < self._nav_ready_at:
            return
        if (
            self._first_goal_sent_at > 0.0
            and not self._saw_waypoint
            and time.monotonic() - self._first_goal_sent_at > self._wp_timeout
        ):
            self.get_logger().error(
                f"FAR 在 {self._wp_timeout:.0f}s 内未收到 /way_point："
                "请确认终端有「V-Graph Initialized」，且 /state_estimation 在发布"
            )
            self._first_goal_sent_at = time.monotonic()
        n = len(self._path.poses)
        end = self._path.poses[-1].pose.position
        if self._dist_to_robot(end.x, end.y) is not None:
            if (self._dist_to_robot(end.x, end.y) or 1e9) <= self._arrival:
                self.get_logger().info("FAR 沿路径到达终点")
                self._navigating = False
                return
        self._advance_progress_along_route()
        goal_i = self._lookahead_goal_index()
        now = time.monotonic()
        if goal_i != self._last_published_wp or now - self._last_publish_t >= self._heartbeat:
            self._publish_goal(goal_i, force=True)


def main() -> None:
    rclpy.init()
    node = FarPathFollowerNode()
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
