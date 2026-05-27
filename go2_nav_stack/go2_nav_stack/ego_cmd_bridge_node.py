#!/usr/bin/env python3
"""将 EGO pos_cmd + 当前路径路点（位置、运动方向）转为 Go2 /cmd_vel。"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException

try:
    from quadrotor_msgs.msg import PositionCommand
except ImportError:
    PositionCommand = None  # type: ignore[misc, assignment]


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class EgoCmdBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("ego_cmd_bridge")
        if PositionCommand is None:
            raise RuntimeError(
                "未找到 quadrotor_msgs，请先编译 ego-planner-swarm: "
                "bash scripts/build_ego_planner_swarm_humble.sh"
            )

        self.declare_parameter("pos_cmd_topic", "/drone_0_planning/pos_cmd")
        self.declare_parameter("ego_path_goal_topic", "/move_base_simple/goal")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 0.15)
        self.declare_parameter("max_vx", 0.35)
        self.declare_parameter("max_wz", 0.5)
        self.declare_parameter("kp_vx", 1.0)
        self.declare_parameter("kp_wz", 1.2)
        self.declare_parameter("min_vx", 0.06)
        self.declare_parameter("near_goal_dist_m", 0.55)
        self.declare_parameter("near_goal_creep_vx", 0.2)
        self.declare_parameter("near_goal_max_wz", 0.35)
        self.declare_parameter("waypoint_yaw_weight", 0.65)
        self.declare_parameter("traj_yaw_weight", 0.35)
        self.declare_parameter("couple_vx_wz", True)
        self.declare_parameter("stop_dist_cmd_m", 0.03)
        self.declare_parameter("stop_dist_path_goal_m", 0.06)
        self.declare_parameter("cmd_stale_sec", 1.0)
        self.declare_parameter("drive_without_pos_cmd", True)
        self.declare_parameter("cmd_rate_hz", 50.0)
        self.declare_parameter("diag_period_sec", 2.0)

        self._max_vx = float(self.get_parameter("max_vx").value)
        self._max_wz = float(self.get_parameter("max_wz").value)
        self._kp_vx = float(self.get_parameter("kp_vx").value)
        self._kp_wz = float(self.get_parameter("kp_wz").value)
        self._min_vx = float(self.get_parameter("min_vx").value)
        self._near_goal_dist = float(self.get_parameter("near_goal_dist_m").value)
        self._near_creep_vx = float(self.get_parameter("near_goal_creep_vx").value)
        self._near_goal_max_wz = float(
            self.get_parameter("near_goal_max_wz").value
        )
        self._wp_yaw_w = float(self.get_parameter("waypoint_yaw_weight").value)
        self._traj_yaw_w = float(self.get_parameter("traj_yaw_weight").value)
        self._couple_vx_wz = bool(self.get_parameter("couple_vx_wz").value)
        self._stop_dist_cmd = float(self.get_parameter("stop_dist_cmd_m").value)
        self._stop_dist_path = float(self.get_parameter("stop_dist_path_goal_m").value)
        self._stale_sec = float(self.get_parameter("cmd_stale_sec").value)
        self._drive_no_cmd = bool(
            self.get_parameter("drive_without_pos_cmd").value
        )
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)

        self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._last_cmd = PositionCommand()
        self._have_cmd = False
        self._cmd_stamp_sec = 0.0
        self._ego_goal: PoseStamped | None = None
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._have_pose = False
        self._warned_no_tf = False
        self._warned_no_cmd = False
        self._warned_no_goal = False
        self._last_zero_reason = ""
        self._last_vx = 0.0
        self._last_wz = 0.0
        self._last_dist_wp = 0.0
        self._last_dist_cmd = 0.0
        self._last_body_to_goal = 0.0
        self._last_yaw_err_wp = 0.0
        self._last_yaw_err_traj = 0.0
        self._last_stale = True
        self._last_use_cmd = False
        self._last_robot_x = 0.0
        self._last_robot_y = 0.0

        topic = str(self.get_parameter("pos_cmd_topic").value)
        self.create_subscription(PositionCommand, topic, self._on_cmd, 20)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("ego_path_goal_topic").value),
            self._on_ego_goal,
            10,
        )
        self._pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        hz = max(1.0, float(self.get_parameter("cmd_rate_hz").value))
        self.create_timer(1.0 / hz, self._on_publish_timer)
        diag_hz = max(0.5, 1.0 / float(self.get_parameter("diag_period_sec").value))
        self.create_timer(1.0 / diag_hz, self._on_diag_timer)
        self.get_logger().info(
            "ego_cmd_bridge: 当前路点朝向 + EGO 局部轨迹 -> /cmd_vel"
        )

    def _on_cmd(self, msg: PositionCommand) -> None:
        self._last_cmd = msg
        self._have_cmd = True
        self._cmd_stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        self._warned_no_cmd = False

    def _on_ego_goal(self, msg: PoseStamped) -> None:
        self._ego_goal = msg

    def _update_robot_pose_from_tf(self) -> bool:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except TransformException as exc:
            if not self._warned_no_tf:
                self.get_logger().warn(
                    f"无 TF {self._map_frame}->{self._base_frame}，/cmd_vel 将为 0: {exc}"
                )
                self._warned_no_tf = True
            self._have_pose = False
            return False

        self._warned_no_tf = False
        t = tf.transform.translation
        r = tf.transform.rotation
        self._robot_x = float(t.x)
        self._robot_y = float(t.y)
        self._robot_yaw = _yaw_from_quat(r.x, r.y, r.z, r.w)
        self._have_pose = True
        return True

    def _current_waypoint(self) -> tuple[float, float, float, float] | None:
        """gx, gy, 路点切线 yaw, 距离。"""
        if self._ego_goal is None:
            return None
        gx = float(self._ego_goal.pose.position.x)
        gy = float(self._ego_goal.pose.position.y)
        o = self._ego_goal.pose.orientation
        wp_yaw = _yaw_from_quat(o.x, o.y, o.z, o.w)
        dist = math.hypot(gx - self._robot_x, gy - self._robot_y)
        return gx, gy, wp_yaw, dist

    def _goal_geometry(
        self, cmd: PositionCommand | None
    ) -> tuple[float, float, float, float, float, float, float, float]:
        """dx,dy, dist_wp, wp_yaw, dist_cmd, yaw_err_wp, yaw_err_traj, body_to_goal。"""
        yaw = self._robot_yaw
        cy, sy = math.cos(yaw), math.sin(yaw)
        wp = self._current_waypoint()
        if wp is not None:
            _, _, wp_yaw, dist_wp = wp
            dx = float(self._ego_goal.pose.position.x) - self._robot_x
            dy = float(self._ego_goal.pose.position.y) - self._robot_y
        elif cmd is not None:
            wp_yaw = float(cmd.yaw)
            dx = float(cmd.position.x) - self._robot_x
            dy = float(cmd.position.y) - self._robot_y
            dist_wp = math.hypot(dx, dy)
        else:
            return 0.0, 0.0, 0.0, yaw, 0.0, 0.0, 0.0, 0.0

        if cmd is not None:
            dx_cmd = float(cmd.position.x) - self._robot_x
            dy_cmd = float(cmd.position.y) - self._robot_y
            dist_cmd = math.hypot(dx_cmd, dy_cmd)
            yaw_err_traj = _normalize_angle(float(cmd.yaw) - yaw)
        else:
            dist_cmd = dist_wp
            yaw_err_traj = 0.0

        yaw_err_wp = _normalize_angle(wp_yaw - yaw)
        body_to_goal = cy * dx + sy * dy
        return (
            dx,
            dy,
            dist_wp,
            wp_yaw,
            dist_cmd,
            yaw_err_wp,
            yaw_err_traj,
            body_to_goal,
        )

    def _apply_min_forward(
        self,
        vx: float,
        wz: float,
        *,
        dist_wp: float,
        body_to_goal: float,
        goal_bearing: float,
        yaw: float,
        along_wp: float,
    ) -> tuple[float, float]:
        if dist_wp <= self._stop_dist_path:
            return vx, wz
        creep = self._near_creep_vx
        if dist_wp >= self._near_goal_dist:
            creep = self._min_vx
        else:
            creep = max(
                self._near_creep_vx,
                self._near_creep_vx
                * min(1.0, dist_wp / max(self._near_goal_dist, 0.1)),
            )
        if abs(vx) >= creep:
            if dist_wp < self._near_goal_dist:
                wz = _clamp(wz, -self._near_goal_max_wz, self._near_goal_max_wz)
            return vx, wz
        along_bearing = math.cos(_normalize_angle(goal_bearing - yaw))
        if abs(body_to_goal) >= 0.02:
            drive_sign = math.copysign(1.0, body_to_goal)
        elif along_bearing >= 0.12:
            drive_sign = 1.0
        elif along_bearing <= -0.12:
            drive_sign = -1.0
        elif along_wp >= 0.12:
            drive_sign = 1.0
        elif along_wp <= -0.12:
            drive_sign = -1.0
        else:
            drive_sign = 1.0
        vx = drive_sign * creep
        if dist_wp < self._near_goal_dist:
            wz = _clamp(wz, -self._near_goal_max_wz, self._near_goal_max_wz)
        return vx, wz

    def _safe_publish(self, twist: Twist) -> None:
        if not rclpy.ok():
            return
        try:
            self._pub.publish(twist)
        except Exception as exc:
            self.get_logger().error(f"[cmd_bridge] publish 失败: {exc}")

    def _on_publish_timer(self) -> None:
        try:
            self._publish_twist()
        except Exception as exc:
            self.get_logger().error(
                f"[cmd_bridge] 定时器异常，节点继续运行: {exc}"
            )
            self._safe_publish(Twist())

    def _on_diag_timer(self) -> None:
        try:
            self._diag_log()
        except Exception as exc:
            self.get_logger().error(f"[cmd_bridge] 诊断日志异常: {exc}")

    def _publish_twist(self) -> None:
        if not self._update_robot_pose_from_tf():
            self._last_zero_reason = "no_tf"
            self._safe_publish(Twist())
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        cmd_stale = (
            not self._have_cmd
            or now_sec - self._cmd_stamp_sec > self._stale_sec
        )
        use_cmd = self._have_cmd and not cmd_stale
        cmd = self._last_cmd if use_cmd else None

        if not use_cmd and not self._drive_no_cmd:
            if not self._warned_no_cmd:
                self.get_logger().warn(
                    f"未收到/已过期 {self.get_parameter('pos_cmd_topic').value}，"
                    "/cmd_vel=0（可设 drive_without_pos_cmd:=true）"
                )
                self._warned_no_cmd = True
            self._last_zero_reason = "no_pos_cmd"
            self._safe_publish(Twist())
            return

        if not use_cmd and self._ego_goal is None and cmd is None:
            if not self._warned_no_goal:
                self.get_logger().warn(
                    "无 pos_cmd 且无 /move_base_simple/goal，/cmd_vel=0"
                )
                self._warned_no_goal = True
            self._last_zero_reason = "no_goal"
            self._safe_publish(Twist())
            return

        if not use_cmd and not self._warned_no_cmd:
            self.get_logger().warn(
                f"未收到有效 {self.get_parameter('pos_cmd_topic').value}，"
                "改按路点 goal 发 /cmd_vel"
            )
            self._warned_no_cmd = True

        yaw = self._robot_yaw
        (
            dx,
            dy,
            dist_wp,
            _wp_yaw,
            dist_cmd,
            yaw_err_wp,
            yaw_err_traj,
            body_to_goal,
        ) = self._goal_geometry(cmd)
        goal_bearing = math.atan2(dy, dx)
        along_wp = math.cos(yaw_err_wp)

        if use_cmd and cmd is not None:
            cy, sy = math.cos(yaw), math.sin(yaw)
            align = max(0.35, math.cos(yaw_err_wp))
            vx_fb = self._kp_vx * body_to_goal * align
            vwx = float(cmd.velocity.x)
            vwy = float(cmd.velocity.y)
            vx_ff = cy * vwx + sy * vwy
            vx = _clamp(vx_fb + vx_ff, -self._max_vx, self._max_vx)
            wz = (
                float(cmd.yaw_dot)
                + self._kp_wz * self._wp_yaw_w * yaw_err_wp
                + self._kp_wz * self._traj_yaw_w * yaw_err_traj
            )
        else:
            vx = _clamp(self._kp_vx * body_to_goal, -self._max_vx, self._max_vx)
            wz = self._kp_wz * (
                self._wp_yaw_w * yaw_err_wp + self._traj_yaw_w * yaw_err_traj
            )

        wz = _clamp(wz, -self._max_wz, self._max_wz)
        vx, wz = self._apply_min_forward(
            vx,
            wz,
            dist_wp=dist_wp,
            body_to_goal=body_to_goal,
            goal_bearing=goal_bearing,
            yaw=yaw,
            along_wp=along_wp,
        )

        # 仅当距路点很近且已基本对准才停（勿用 dist_cmd，EGO 轨迹点常在脚下）
        if dist_wp < self._stop_dist_path and abs(body_to_goal) < 0.03:
            vx = 0.0
            self._last_zero_reason = "at_goal"
        else:
            self._last_zero_reason = "ok" if use_cmd else "goal_only"

        if self._couple_vx_wz:
            vx, wz = self._couple_planar_speed(vx, wz)

        vx = _clamp(vx, -self._max_vx, self._max_vx)

        out = Twist()
        out.linear.x = vx
        out.angular.z = wz
        self._safe_publish(out)
        self._last_vx = vx
        self._last_wz = wz
        self._last_dist_wp = dist_wp
        self._last_yaw_err_wp = yaw_err_wp
        self._last_yaw_err_traj = yaw_err_traj
        self._last_stale = cmd_stale
        self._last_body_to_goal = body_to_goal
        self._last_use_cmd = use_cmd
        self._last_dist_cmd = dist_cmd
        self._last_robot_x = self._robot_x
        self._last_robot_y = self._robot_y

    def _couple_planar_speed(self, vx: float, wz: float) -> tuple[float, float]:
        turn_ratio = min(1.0, abs(wz) / max(self._max_wz, 1e-3))
        vx_cap = self._max_vx * max(0.4, 1.0 - 0.55 * turn_ratio)
        vx = _clamp(vx, -vx_cap, vx_cap)
        speed_ratio = min(1.0, abs(vx) / max(self._max_vx, 1e-3))
        wz_cap = self._max_wz * max(0.5, 1.0 - 0.5 * speed_ratio)
        wz = _clamp(wz, -wz_cap, wz_cap)
        return vx, wz

    def _diag_log(self) -> None:
        if not self._have_pose:
            self.get_logger().warn(
                "[cmd_bridge] 无 TF，未发速度",
                throttle_duration_sec=2.0,
            )
            return
        self.get_logger().info(
            f"[cmd_bridge] publish vx={getattr(self, '_last_vx', 0.0):.3f} "
            f"wz={getattr(self, '_last_wz', 0.0):.3f} "
            f"robot=({getattr(self, '_last_robot_x', 0.0):.2f},"
            f"{getattr(self, '_last_robot_y', 0.0):.2f}) "
            f"dist_wp={getattr(self, '_last_dist_wp', 0.0):.2f}m "
            f"dist_cmd={getattr(self, '_last_dist_cmd', 0.0):.2f}m "
            f"body_goal={getattr(self, '_last_body_to_goal', 0.0):.3f} "
            f"yaw_err_wp={math.degrees(getattr(self, '_last_yaw_err_wp', 0.0)):.0f}° "
            f"yaw_err_traj={math.degrees(getattr(self, '_last_yaw_err_traj', 0.0)):.0f}° "
            f"mode={getattr(self, '_last_zero_reason', '?')} "
            f"pos_cmd={getattr(self, '_last_use_cmd', False)} "
            f"stale={getattr(self, '_last_stale', False)} "
            f"max_vx={self._max_vx} creep={self._near_creep_vx} "
            f"stop_wp<{self._stop_dist_path:.2f}m have_goal={self._ego_goal is not None}",
            throttle_duration_sec=1.0,
        )


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = EgoCmdBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"ego_cmd_bridge 启动失败: {exc}", flush=True)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
