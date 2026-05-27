#!/usr/bin/env python3
"""重定位：更新 map->odom（运动时靠 FAST-LIO2 /lio_odom_raw + tf_manager odom->base）。

模式（由参数决定）：
  - manual（默认一键启动）：Qt/网页「当前姿态」点选 -> /initialpose -> ICP -> /relocalization/pose
  - auto_startup：开机 Scan Context + ICP
  - service：/relocalization_manager/relocalize 触发 Scan Context
  - tf_lost：导航中 TF 丢失边沿（可关）
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class RelocalizationManager(Node):
    _ALLOWED_REASONS = frozenset({"startup", "manual", "tf_lost", "service"})

    def __init__(self) -> None:
        super().__init__("relocalization_manager")
        self.declare_parameter("auto_reloc_on_start", False)
        self.declare_parameter("accept_manual_initialpose", True)
        self.declare_parameter("startup_delay_sec", 10.0)
        self.declare_parameter("reloc_on_tf_lost_when_navigating", False)
        self.declare_parameter("tf_lost_reloc_cooldown_sec", 30.0)
        self.declare_parameter("accept_continuous_icp", True)
        self.declare_parameter("lio_weak_reloc_sec", 5.0)
        self.declare_parameter("lio_weak_eff_threshold", 8)
        self.declare_parameter("lio_weak_reloc_cooldown_sec", 15.0)
        self.declare_parameter("block_reloc_during_spin", True)
        self.declare_parameter("spin_block_reloc_wz", 0.55)
        self.declare_parameter("spin_reloc_settle_sec", 2.0)

        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._reloc_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/relocalization/pose", 10
        )
        self._status_pub = self.create_publisher(String, "/relocalization/status", 10)
        self._ok_pub = self.create_publisher(Bool, "/relocalization/ok", 10)
        self.create_subscription(
            String, "/scan_context/last_query", self._on_sc_result, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/icp_result", self._on_icp_result, 10
        )
        self.create_subscription(String, "/navigation/state", self._on_nav_state, 10)
        self.create_subscription(
            Int32, "/fastlio/effective_points", self._on_lio_effective, 10
        )
        if bool(self.get_parameter("accept_manual_initialpose").value):
            self.create_subscription(
                PoseWithCovarianceStamped,
                "/initialpose",
                self._on_manual_initialpose,
                10,
            )
        self._sc_client = self.create_client(Trigger, "/scan_context_manager/query")
        self.create_service(Trigger, "~/relocalize", self._on_reloc)

        self._awaiting_icp = False
        self._reloc_busy = False
        self._reloc_locked = False
        self._nav_state = "IDLE"
        self._prev_nav_state = "IDLE"
        self._startup_done = False
        self._last_reloc_time = 0.0
        self._lio_weak_since: float | None = None
        self._last_lio_weak_reloc = 0.0
        self._last_wz = 0.0
        self._last_odom_mono = 0.0
        self._spin_settle_until = 0.0
        self._block_spin_reloc = bool(self.get_parameter("block_reloc_during_spin").value)
        self._spin_block_wz = float(self.get_parameter("spin_block_reloc_wz").value)
        self._spin_settle_sec = float(self.get_parameter("spin_reloc_settle_sec").value)

        self.create_subscription(Odometry, "/lio_odom_raw", self._on_lio_odom, 20)

        if bool(self.get_parameter("auto_reloc_on_start").value):
            delay = float(self.get_parameter("startup_delay_sec").value)
            self.create_timer(delay, self._startup_reloc_once)
        else:
            self._ok_pub.publish(Bool(data=False))
            self._status_pub.publish(String(data="WAIT_MANUAL_POSE"))
            self.get_logger().info(
                "relocalization: manual mode — use Qt map_viewer "
                "「当前姿态」点位置+朝向，发布 /initialpose 后等待 ICP"
            )
        if bool(self.get_parameter("auto_reloc_on_start").value):
            self.get_logger().info(
                "relocalization_manager: auto startup ScanContext+ICP enabled"
            )
        elif bool(self.get_parameter("accept_manual_initialpose").value):
            self.get_logger().info(
                "relocalization_manager: manual /initialpose -> ICP -> /relocalization/pose"
            )

    def _on_lio_odom(self, msg: Odometry) -> None:
        self._last_wz = float(msg.twist.twist.angular.z)
        self._last_odom_mono = time.monotonic()

    def _is_spin_protected(self) -> bool:
        if not self._block_spin_reloc:
            return False
        now = time.monotonic()
        if now < self._spin_settle_until:
            return True
        if now - self._last_odom_mono > 0.6:
            return False
        if abs(self._last_wz) >= self._spin_block_wz:
            self._spin_settle_until = now + self._spin_settle_sec
            return True
        return False

    def _on_manual_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        """Qt「当前姿态」/ RViz 2D Pose：仅 ICP，不跑 Scan Context。"""
        if self._reloc_busy:
            return
        self._reloc_locked = False
        self._ok_pub.publish(Bool(data=False))
        p = msg.pose.pose.position
        self._awaiting_icp = True
        self._status_pub.publish(String(data="MANUAL_ICP: awaiting /icp_result"))
        self.get_logger().info(
            f"Manual /initialpose ({p.x:.2f},{p.y:.2f}) -> waiting NDT/ICP /icp_result"
        )

    def _startup_reloc_once(self) -> None:
        if self._startup_done:
            return
        self._startup_done = True
        self._trigger_reloc("startup")

    def _on_nav_state(self, msg: String) -> None:
        state = msg.data.strip()
        prev = self._nav_state
        self._prev_nav_state = prev
        self._nav_state = state
        if state != "TF_LOST":
            return
        if prev == "TF_LOST":
            return
        if not bool(self.get_parameter("reloc_on_tf_lost_when_navigating").value):
            return
        if self._reloc_busy:
            return
        if prev not in (
            "LOCAL_NAVIGATION",
            "GLOBAL_PLAN_READY",
            "AVOIDING",
            "SLOW_DOWN",
        ):
            return
        self._trigger_reloc("tf_lost")

    def _on_lio_effective(self, msg: Int32) -> None:
        if not self._reloc_locked or self._reloc_busy:
            self._lio_weak_since = None
            return
        if self._is_spin_protected():
            self._lio_weak_since = None
            return
        threshold = int(self.get_parameter("lio_weak_eff_threshold").value)
        need_weak_sec = float(self.get_parameter("lio_weak_reloc_sec").value)
        cooldown = float(self.get_parameter("lio_weak_reloc_cooldown_sec").value)
        now = time.monotonic()
        if msg.data >= threshold:
            self._lio_weak_since = None
            return
        if self._lio_weak_since is None:
            self._lio_weak_since = now
            return
        if now - self._lio_weak_since < need_weak_sec:
            return
        if now - self._last_lio_weak_reloc < cooldown:
            return
        self._lio_weak_since = None
        self._last_lio_weak_reloc = now
        self.get_logger().warn(
            f"LIO eff={msg.data}<{threshold} for {need_weak_sec:.0f}s -> ScanContext+ICP"
        )
        self._trigger_reloc("service")

    def _on_reloc(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self._trigger_reloc("service")
        res.success = True
        res.message = "Scan Context + NDT/ICP relocalization triggered"
        return res

    def _trigger_reloc(self, reason: str) -> None:
        if reason not in self._ALLOWED_REASONS:
            self.get_logger().warn(f"relocalization ignored: unknown reason '{reason}'")
            return
        if reason != "startup" and self._is_spin_protected():
            self.get_logger().warn(
                f"转圈中/刚停转：跳过自动重定位 ({reason})",
                throttle_duration_sec=2.0,
            )
            return
        if reason == "startup" and self._reloc_locked:
            self.get_logger().info("startup relocalization skipped (already locked)")
            return
        if self._reloc_busy:
            self.get_logger().warn(f"relocalization already in progress ({reason})")
            return
        if reason == "tf_lost":
            cooldown = float(self.get_parameter("tf_lost_reloc_cooldown_sec").value)
            now = time.monotonic()
            if now - self._last_reloc_time < cooldown:
                self.get_logger().warn(
                    f"tf_lost relocalization skipped (cooldown {cooldown:.0f}s)"
                )
                return
        if not self._sc_client.wait_for_service(timeout_sec=2.0):
            self._publish_status(False, "scan_context query service not ready")
            return

        self._reloc_busy = True
        self._awaiting_icp = False
        self._last_reloc_time = time.monotonic()
        self._status_pub.publish(String(data=f"RELOCALIZING:{reason}"))
        self.get_logger().info(f"relocalization triggered ({reason})")
        fut = self._sc_client.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: self._sc_done(f, reason))

    def _sc_done(self, fut, reason: str) -> None:
        try:
            res = fut.result()
        except Exception as ex:
            self._reloc_busy = False
            self.get_logger().error(f"scan_context query failed: {ex}")
            self._publish_status(False, str(ex))
            return
        if not res or not res.success:
            self._reloc_busy = False
            self._publish_status(False, res.message if res else "query failed")
            return
        self.get_logger().info(f"scan_context ok ({reason}): {res.message[:120]}")

    def _on_sc_result(self, msg: String) -> None:
        if not self._reloc_busy:
            return
        text = msg.data.strip()
        if not text:
            self._reloc_busy = False
            return
        first = text.split(";")[0].strip()
        try:
            vals = {}
            for token in first.replace(",", " ").split():
                if "=" in token:
                    k, v = token.split("=", 1)
                    vals[k.strip()] = float(v)
            x = vals["x"]
            y = vals["y"]
            yaw = vals["yaw"]
        except (KeyError, ValueError):
            self._reloc_busy = False
            self.get_logger().warn(f"parse scan_context result failed: {text}")
            self._publish_status(False, "parse failed")
            return

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation = _yaw_to_quat(yaw)
        self._initial_pose_pub.publish(pose)
        self._awaiting_icp = True
        self._publish_status(True, f"SC coarse pose ({x:.2f},{y:.2f}) awaiting NDT/ICP")
        self.get_logger().info(
            f"Scan Context -> /initialpose ({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.1f}°, "
            "waiting NDT/ICP /icp_result"
        )

    def _on_icp_result(self, msg: PoseWithCovarianceStamped) -> None:
        accept_continuous = bool(self.get_parameter("accept_continuous_icp").value)
        if not self._awaiting_icp and not (accept_continuous and self._reloc_locked):
            return
        first_lock = not self._reloc_locked
        if not first_lock and self._is_spin_protected():
            self.get_logger().warn(
                "转圈中：忽略 ICP 连续修正，避免 map->odom 被拉偏",
                throttle_duration_sec=2.0,
            )
            return
        self._awaiting_icp = False
        self._reloc_busy = False
        self._reloc_locked = True
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id or "map"
        if out.header.frame_id != "map":
            self.get_logger().warn(
                f"/icp_result frame={out.header.frame_id}，强制为 map"
            )
            out.header.frame_id = "map"
        out.pose = msg.pose
        self._reloc_pose_pub.publish(out)
        self._ok_pub.publish(Bool(data=True))
        self._status_pub.publish(String(data="RELOCALIZED"))
        p = out.pose.pose.position
        yaw = math.atan2(
            2.0 * (out.pose.pose.orientation.w * out.pose.pose.orientation.z),
            1.0 - 2.0 * (out.pose.pose.orientation.z**2),
        )
        tag = "NDT/ICP locked" if first_lock else "ICP refine"
        self.get_logger().info(
            f"{tag} map->odom via /relocalization/pose "
            f"({p.x:.2f},{p.y:.2f}) yaw={math.degrees(yaw):.1f}°"
        )

    def _publish_status(self, ok: bool, message: str) -> None:
        self._status_pub.publish(String(data=message))
        if not ok:
            self._ok_pub.publish(Bool(data=False))


def main() -> None:
    rclpy.init()
    node = RelocalizationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
