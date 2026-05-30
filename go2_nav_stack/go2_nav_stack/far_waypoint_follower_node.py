#!/usr/bin/env python3
"""跟踪 FAR Planner 发布的 /way_point，输出四足 /cmd_vel。"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool, String
import tf2_ros
from tf2_ros import TransformException


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _norm_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class FarWaypointFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("far_waypoint_follower")
        self.declare_parameter("waypoint_topic", "/way_point")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 0.25)
        self.declare_parameter("max_vx", 0.40)
        self.declare_parameter("max_wz", 0.32)
        self.declare_parameter("kp_vx", 0.95)
        self.declare_parameter("kp_wz", 0.55)
        self.declare_parameter("min_vx", 0.10)
        self.declare_parameter("arrival_xy_m", 0.35)
        self.declare_parameter("forward_only", True)
        self.declare_parameter("cmd_rate_hz", 30.0)
        self.declare_parameter("status_topic", "/local_planner/status")
        self.declare_parameter("exec_path_topic", "/far_exec_path")
        self.declare_parameter("trail_max_points", 120)

        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self._max_vx = float(self.get_parameter("max_vx").value)
        self._max_wz = float(self.get_parameter("max_wz").value)
        self._kp_vx = float(self.get_parameter("kp_vx").value)
        self._kp_wz = float(self.get_parameter("kp_wz").value)
        self._min_vx = float(self.get_parameter("min_vx").value)
        self._arrival = float(self.get_parameter("arrival_xy_m").value)
        self._forward_only = bool(self.get_parameter("forward_only").value)

        self._pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self._path_pub = self.create_publisher(
            Path, str(self.get_parameter("exec_path_topic").value), 10
        )

        wp_topic = str(self.get_parameter("waypoint_topic").value)
        self.create_subscription(PointStamped, wp_topic, self._on_waypoint, 10)
        self.create_subscription(Bool, "/far_reach_goal_status", self._on_reach, 10)
        self.create_subscription(Bool, "/stop_navigation", self._on_stop, 10)

        self._buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._listener = tf2_ros.TransformListener(self._buffer, self)

        self._wp: PointStamped | None = None
        self._reach_goal = False
        self._active = False
        self._trail: list[tuple[float, float]] = []
        self._trail_max = int(self.get_parameter("trail_max_points").value)

        hz = float(self.get_parameter("cmd_rate_hz").value)
        self.create_timer(1.0 / max(hz, 5.0), self._tick)
        self.get_logger().info(f"far_waypoint_follower: 跟踪 {wp_topic} -> /cmd_vel")

    def _on_waypoint(self, msg: PointStamped) -> None:
        self._wp = msg
        self._reach_goal = False
        self._active = True

    def _on_reach(self, msg: Bool) -> None:
        if msg.data:
            self._reach_goal = True
            self._active = False
            self._pub.publish(Twist())
            self._status_pub.publish(String(data="GOAL_REACHED"))

    def _on_stop(self, msg: Bool) -> None:
        if msg.data:
            self._active = False
            self._wp = None
            self._pub.publish(Twist())
            self._status_pub.publish(String(data="CLEAR"))

    def _pose(self) -> tuple[float, float, float] | None:
        try:
            tf = self._buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return float(t.x), float(t.y), _yaw_from_quat(q.x, q.y, q.z, q.w)

    def _tick(self) -> None:
        pose = self._pose()
        if pose is None:
            self._pub.publish(Twist())
            return
        rx, ry, yaw = pose

        if self._reach_goal:
            return
        if not self._active or self._wp is None:
            self._status_pub.publish(String(data="CLEAR"))
            self._pub.publish(Twist())
            return

        gx = float(self._wp.point.x)
        gy = float(self._wp.point.y)
        dx, dy = gx - rx, gy - ry
        dist = math.hypot(dx, dy)
        if dist < self._arrival:
            self._pub.publish(Twist())
            self._status_pub.publish(String(data="GOAL_REACHED"))
            return

        bearing = math.atan2(dy, dx)
        yaw_err = _norm_angle(bearing - yaw)
        cy, sy = math.cos(yaw), math.sin(yaw)
        body_x = cy * dx + sy * dy

        vx = _clamp(self._kp_vx * body_x, -self._max_vx, self._max_vx)
        wz = _clamp(self._kp_wz * yaw_err, -self._max_wz, self._max_wz)

        if self._forward_only and vx < 0.0:
            vx = 0.0
            wz = _clamp(self._kp_wz * yaw_err, -self._max_wz, self._max_wz)

        if abs(vx) < self._min_vx and dist > self._arrival:
            vx = math.copysign(self._min_vx, body_x if abs(body_x) > 1e-3 else 1.0)

        out = Twist()
        out.linear.x = vx
        out.angular.z = wz
        self._pub.publish(out)
        self._status_pub.publish(String(data="FAR_FOLLOWING"))

        self._trail.append((rx, ry))
        if len(self._trail) > self._trail_max:
            self._trail = self._trail[-self._trail_max :]
        path = Path()
        path.header.frame_id = self._map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in self._trail:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._path_pub.publish(path)


def main() -> None:
    rclpy.init()
    node = FarWaypointFollowerNode()
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
