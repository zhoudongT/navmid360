#!/usr/bin/env python3
"""全局路径管理：插值、进度跟踪、走廊局部目标、供 MPPI / 重定位使用。"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, String
from visualization_msgs.msg import Marker
import tf2_ros
from tf2_ros import TransformException


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    h = yaw * 0.5
    return (0.0, 0.0, math.sin(h), math.cos(h))


def _normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class GlobalPathManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("global_path_manager")
        self.declare_parameter("planned_path_topic", "/planned_path")
        self.declare_parameter("processed_path_topic", "/global_path_processed")
        self.declare_parameter("local_target_topic", "/local_target_pose")
        self.declare_parameter("corridor_marker_topic", "/path_corridor_marker")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 0.25)
        self.declare_parameter("path_spacing_m", 0.10)
        self.declare_parameter("lookahead_normal_m", 1.5)
        self.declare_parameter("lookahead_stair_m", 0.4)
        self.declare_parameter("corridor_width_normal_m", 0.4)
        self.declare_parameter("corridor_width_stair_m", 0.4)
        self.declare_parameter("corridor_publish_ahead_m", 10.0)
        self.declare_parameter("corridor_publish_behind_m", 2.0)
        self.declare_parameter("max_lateral_error_m", 0.8)
        self.declare_parameter("goal_reached_m", 0.35)
        self.declare_parameter("tick_hz", 10.0)
        self.declare_parameter("default_speed_limit", 0.40)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._robot_frame = str(self.get_parameter("robot_frame").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self._spacing = float(self.get_parameter("path_spacing_m").value)
        self._lookahead_normal = float(self.get_parameter("lookahead_normal_m").value)
        self._lookahead_stair = float(self.get_parameter("lookahead_stair_m").value)
        self._corridor_normal = float(self.get_parameter("corridor_width_normal_m").value)
        self._corridor_stair = float(self.get_parameter("corridor_width_stair_m").value)
        self._pub_ahead = float(self.get_parameter("corridor_publish_ahead_m").value)
        self._pub_behind = float(self.get_parameter("corridor_publish_behind_m").value)
        self._max_lat = float(self.get_parameter("max_lateral_error_m").value)
        self._goal_reached = float(self.get_parameter("goal_reached_m").value)
        self._default_speed = float(self.get_parameter("default_speed_limit").value)

        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        proc_topic = str(self.get_parameter("processed_path_topic").value)
        self._path_pub = self.create_publisher(Path, proc_topic, path_qos)
        self._target_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("local_target_topic").value), 10
        )
        self._segment_pub = self.create_publisher(String, "/current_path_segment", 10)
        self._mode_pub = self.create_publisher(String, "/current_behavior_mode", 10)
        self._progress_pub = self.create_publisher(Int32, "/path_progress", 10)
        self._status_pub = self.create_publisher(String, "/global_path_manager/status", 10)
        self._corridor_pub = self.create_publisher(
            Marker, str(self.get_parameter("corridor_marker_topic").value), 10
        )

        self._buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._listener = tf2_ros.TransformListener(self._buffer, self)

        self._raw: Path | None = None
        self._dense: list[tuple[float, float, float, float, str, float, float]] = []
        # x, y, z, yaw, type, speed_limit, corridor_width
        self._progress = 0
        self._navigating = False
        self._behavior = "IDLE"

        in_topic = str(self.get_parameter("planned_path_topic").value)
        self.create_subscription(Path, in_topic, self._on_raw_path, path_qos)
        self.create_subscription(Bool, "/start_navigation", self._on_start, 10)
        self.create_subscription(Bool, "/stop_navigation", self._on_stop, 10)

        hz = float(self.get_parameter("tick_hz").value)
        self.create_timer(1.0 / max(hz, 2.0), self._tick)
        self.get_logger().info(
            f"global_path_manager: {in_topic} -> {proc_topic}, "
            f"走廊宽={self._corridor_normal}m 前视={self._lookahead_normal}m"
        )

    def _on_raw_path(self, msg: Path) -> None:
        self._raw = msg
        self._dense = self._interpolate_path(msg)
        if self._navigating and self._dense:
            self._progress = self._find_nearest_index()
        elif self._dense:
            self._publish_preview_segment()

    def _on_start(self, msg: Bool) -> None:
        if not msg.data:
            return
        if not self._dense:
            self.get_logger().warn("start_navigation: 无全局路径")
            return
        self._navigating = True
        self._behavior = "NAVIGATING"
        self._progress = self._find_nearest_index()
        self._publish_active_segment()
        self.get_logger().info(
            f"路径管理启动: {len(self._dense)} 点, progress={self._progress}"
        )

    def _on_stop(self, msg: Bool) -> None:
        if msg.data:
            self._navigating = False
            self._behavior = "IDLE"

    def _robot_xy_yaw(self) -> tuple[float, float, float] | None:
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
        return float(t.x), float(t.y), _yaw_from_quat(r.x, r.y, r.z, r.w)

    def _interpolate_path(
        self, path: Path
    ) -> list[tuple[float, float, float, float, str, float, float]]:
        if not path.poses:
            return []
        out: list[tuple[float, float, float, float, str, float, float]] = []
        poses = path.poses
        n = len(poses)

        def pt(i: int) -> tuple[float, float, float]:
            p = poses[i].pose.position
            return float(p.x), float(p.y), float(p.z)

        def yaw_at(i: int) -> float:
            o = poses[i].pose.orientation
            qn = math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w)
            if qn > 0.1:
                return _yaw_from_quat(o.x, o.y, o.z, o.w)
            if i < n - 1:
                a, b = pt(i), pt(i + 1)
            elif i > 0:
                a, b = pt(i - 1), pt(i)
            else:
                return 0.0
            return math.atan2(b[1] - a[1], b[0] - a[0])

        for i in range(n):
            x, y, z = pt(i)
            yaw = yaw_at(i)
            seg_type = "normal"
            speed = self._default_speed
            width = self._corridor_normal
            out.append((x, y, z, yaw, seg_type, speed, width))

        if len(out) < 2 or self._spacing <= 1e-3:
            return out

        dense: list[tuple[float, float, float, float, str, float, float]] = [out[0]]
        for i in range(n - 1):
            x0, y0, z0, y0a, t0, s0, w0 = out[i]
            x1, y1, z1, y1a, t1, s1, w1 = out[i + 1]
            seg_len = math.hypot(x1 - x0, y1 - y0)
            if seg_len < 1e-4:
                continue
            steps = max(1, int(math.ceil(seg_len / self._spacing)))
            for k in range(1, steps + 1):
                u = k / steps
                dense.append(
                    (
                        x0 + u * (x1 - x0),
                        y0 + u * (y1 - y0),
                        z0 + u * (z1 - z0),
                        _normalize_angle(y0a + u * _normalize_angle(y1a - y0a)),
                        t1 if u > 0.5 else t0,
                        s0 + u * (s1 - s0),
                        w0 + u * (w1 - w0),
                    )
                )
        return dense

    def _find_nearest_index(self) -> int:
        if not self._dense:
            return 0
        pose = self._robot_xy_yaw()
        if pose is None:
            return self._progress
        rx, ry, _ = pose
        best_i, best_d = self._progress, 1e9
        search_lo = max(0, self._progress - 80)
        search_hi = min(len(self._dense), self._progress + 120)
        for i in range(search_lo, search_hi):
            x, y, _, _, _, _, _ = self._dense[i]
            d = math.hypot(x - rx, y - ry)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def _advance_progress(self) -> None:
        if not self._dense:
            return
        pose = self._robot_xy_yaw()
        if pose is None:
            return
        rx, ry, _ = pose
        pass_r = max(0.25, self._goal_reached * 0.85)
        while self._progress < len(self._dense) - 1:
            x, y, _, _, _, _, _ = self._dense[self._progress]
            d_cur = math.hypot(x - rx, y - ry)
            if d_cur < pass_r:
                self._progress += 1
                continue
            xn, yn, _, _, _, _, _ = self._dense[self._progress + 1]
            if math.hypot(xn - rx, yn - ry) + 0.08 < d_cur:
                self._progress += 1
                continue
            break

    def _lookahead_index(self) -> int:
        if not self._dense:
            return 0
        start = min(max(0, self._progress), len(self._dense) - 1)
        _, _, _, _, seg_type, _, _ = self._dense[start]
        lookahead = (
            self._lookahead_stair if seg_type.startswith("stair") else self._lookahead_normal
        )
        acc = 0.0
        target = start
        for j in range(start, len(self._dense) - 1):
            x0, y0, _, _, _, _, _ = self._dense[j]
            x1, y1, _, _, _, _, _ = self._dense[j + 1]
            acc += math.hypot(x1 - x0, y1 - y0)
            target = j + 1
            if acc >= lookahead:
                break
        if target == start and start < len(self._dense) - 1:
            target = start + 1
        return min(target, len(self._dense) - 1)

    def _segment_window(self) -> tuple[int, int]:
        if not self._dense:
            return 0, 0
        rx, ry, _ = self._robot_xy_yaw() or (0.0, 0.0, 0.0)
        behind = self._pub_behind
        ahead = self._pub_ahead
        lo, hi = self._progress, self._progress
        acc_b = 0.0
        while lo > 0 and acc_b < behind:
            x0, y0, _, _, _, _, _ = self._dense[lo]
            x1, y1, _, _, _, _, _ = self._dense[lo - 1]
            acc_b += math.hypot(x1 - x0, y1 - y0)
            lo -= 1
        acc_a = 0.0
        while hi < len(self._dense) - 1 and acc_a < ahead:
            x0, y0, _, _, _, _, _ = self._dense[hi]
            x1, y1, _, _, _, _, _ = self._dense[hi + 1]
            acc_a += math.hypot(x1 - x0, y1 - y0)
            hi += 1
        return lo, hi + 1

    def _publish_processed(self, lo: int, hi: int) -> None:
        if not self._dense or lo >= hi:
            return
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._map_frame
        for i in range(lo, hi):
            x, y, z, yaw, _, _, _ = self._dense[i]
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            qx, qy, qz, qw = _yaw_to_quat(yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            out.poses.append(ps)
        self._path_pub.publish(out)

    def _publish_local_target(self, idx: int) -> None:
        if not self._dense:
            return
        idx = min(max(0, idx), len(self._dense) - 1)
        x, y, z, yaw, seg_type, speed, width = self._dense[idx]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._target_pub.publish(msg)
        seg = String()
        seg.data = f"idx={idx} type={seg_type} speed={speed:.2f} corridor={width:.2f}"
        self._segment_pub.publish(seg)
        mode = String()
        mode.data = (
            "CLIMBING_STAIR"
            if seg_type.startswith("stair") and seg_type != "stair_entry"
            else "NAVIGATING"
            if self._navigating
            else "IDLE"
        )
        self._mode_pub.publish(mode)

    def _publish_corridor_marker(self, lo: int, hi: int) -> None:
        if not self._dense or lo >= hi - 1:
            return
        left_pts: list[Point] = []
        right_pts: list[Point] = []
        for i in range(lo, hi - 1):
            x0, y0, z0, yaw0, _, _, w0 = self._dense[i]
            x1, y1, z1, _, _, _, w1 = self._dense[i + 1]
            yaw = math.atan2(y1 - y0, x1 - x0)
            half = 0.5 * (w0 + w1)
            nx, ny = -math.sin(yaw), math.cos(yaw)
            z = 0.5 * (z0 + z1)
            p_l = Point(x=x0 + nx * half, y=y0 + ny * half, z=z)
            p_r = Point(x=x0 - nx * half, y=y0 - ny * half, z=z)
            left_pts.append(p_l)
            right_pts.append(p_r)
        if not left_pts:
            return
        mk = Marker()
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.header.frame_id = self._map_frame
        mk.ns = "path_corridor"
        mk.id = 0
        mk.type = Marker.LINE_LIST
        mk.action = Marker.ADD
        mk.scale.x = 0.04
        mk.color.r = 0.2
        mk.color.g = 0.85
        mk.color.b = 0.3
        mk.color.a = 0.55
        for pl, pr in zip(left_pts, right_pts):
            mk.points.append(pl)
            mk.points.append(pr)
        self._corridor_pub.publish(mk)

    def _lateral_error(self) -> float | None:
        if not self._dense:
            return None
        pose = self._robot_xy_yaw()
        if pose is None:
            return None
        rx, ry, _ = pose
        idx = min(max(0, self._progress), len(self._dense) - 1)
        x, y, _, yaw, _, _, _ = self._dense[idx]
        dx, dy = rx - x, ry - y
        return abs(-math.sin(yaw) * dx + math.cos(yaw) * dy)

    def _publish_preview_segment(self) -> None:
        """未开始导航时也发布路径段，供 MPPI/RViz 预览。"""
        if not self._dense:
            return
        lo, hi = self._segment_window()
        self._publish_processed(lo, hi)
        self._publish_corridor_marker(lo, hi)

    def _publish_active_segment(self) -> None:
        if not self._dense:
            return
        lo, hi = self._segment_window()
        self._publish_processed(lo, hi)
        target_i = self._lookahead_index()
        self._publish_local_target(target_i)
        self._publish_corridor_marker(lo, hi)

    def _tick(self) -> None:
        if not self._dense:
            return
        prog = Int32()
        prog.data = self._progress
        self._progress_pub.publish(prog)

        if not self._navigating:
            self._publish_preview_segment()
            st = String()
            st.data = "PATH_LOADED"
            self._status_pub.publish(st)
            return

        self._advance_progress()
        lat = self._lateral_error()
        end_x, end_y, _, _, _, _, _ = self._dense[-1]
        pose = self._robot_xy_yaw()
        if pose is not None:
            rx, ry, _ = pose
            if math.hypot(end_x - rx, end_y - ry) <= self._goal_reached:
                self._navigating = False
                self._behavior = "IDLE"
                st = String()
                st.data = "GOAL_REACHED"
                self._status_pub.publish(st)
                return

        self._publish_active_segment()

        st = String()
        if lat is not None and lat > self._max_lat:
            st.data = f"OFF_CORRIDOR lat={lat:.2f}m"
        else:
            st.data = f"ON_PATH progress={self._progress}/{len(self._dense)-1}"
        self._status_pub.publish(st)


def main() -> None:
    rclpy.init()
    node = GlobalPathManagerNode()
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
