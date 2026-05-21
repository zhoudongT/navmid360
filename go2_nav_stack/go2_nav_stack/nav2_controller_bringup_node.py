#!/usr/bin/env python3
"""先激活独立 local_costmap（RViz 局部图），再激活 controller_server（MPPI）。"""

from __future__ import annotations

import os
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class Nav2ControllerBringupNode(Node):
    def __init__(self) -> None:
        super().__init__("nav2_controller_bringup")
        self.declare_parameter("costmap_node", "costmap/costmap")
        self.declare_parameter("controller_node", "controller_server")
        self.declare_parameter("startup_delay_sec", 5.0)
        self.declare_parameter("node_wait_timeout_sec", 15.0)
        self.declare_parameter("costmap_configure_timeout_sec", 60.0)
        self.declare_parameter("costmap_activate_timeout_sec", 30.0)
        self.declare_parameter("configure_timeout_sec", 180.0)
        self.declare_parameter("activate_timeout_sec", 90.0)
        self.declare_parameter("retry_period_sec", 30.0)

        self._costmap_node = str(self.get_parameter("costmap_node").value).lstrip("/")
        self._ctrl_node = str(self.get_parameter("controller_node").value).lstrip("/")
        self._startup_delay = float(self.get_parameter("startup_delay_sec").value)
        self._node_wait_t = float(self.get_parameter("node_wait_timeout_sec").value)
        self._costmap_cfg_t = float(self.get_parameter("costmap_configure_timeout_sec").value)
        self._costmap_act_t = float(self.get_parameter("costmap_activate_timeout_sec").value)
        self._ctrl_cfg_t = float(self.get_parameter("configure_timeout_sec").value)
        self._ctrl_act_t = float(self.get_parameter("activate_timeout_sec").value)
        self._retry_period = float(self.get_parameter("retry_period_sec").value)
        self._costmap_ok = False
        self._ctrl_ok = False

        status_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        self._status_pub = self.create_publisher(
            String, "/nav2_controller/status", status_qos
        )
        self._costmap_pub = self.create_publisher(Bool, "/local_costmap/available", 10)

        self.get_logger().info(
            f"bringup: {self._startup_delay:.0f}s 后先激活 {self._costmap_node}，再 {self._ctrl_node}"
        )
        threading.Thread(target=self._bringup_loop, daemon=True).start()

    def _publish(self, text: str) -> None:
        self._status_pub.publish(String(data=text))

    def _lifecycle_state(self, node: str) -> str:
        cmd = ["ros2", "lifecycle", "get", f"/{node}"]
        try:
            proc = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=5.0,
                env=os.environ.copy(),
            )
        except subprocess.TimeoutExpired:
            return ""
        return ((proc.stdout or "") + (proc.stderr or "")).strip().lower()

    def _node_listed(self, node: str) -> bool:
        name = node if node.startswith("/") else f"/{node}"
        try:
            proc = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True,
                text=True,
                timeout=5.0,
                env=os.environ.copy(),
            )
        except subprocess.TimeoutExpired:
            return False
        return name in (proc.stdout or "")

    def _node_available(self, node: str) -> bool:
        if self._node_listed(node):
            return True
        state = self._lifecycle_state(node)
        if not state:
            return False
        return "not found" not in state and "does not exist" not in state

    def _wait_for_node(self, node: str) -> bool:
        deadline = time.time() + self._node_wait_t
        while time.time() < deadline and rclpy.ok():
            if self._node_available(node):
                return True
            time.sleep(0.5)
        self.get_logger().warn(
            f"/{node} 在 {self._node_wait_t:.0f}s 内未出现（检查 nav2_mppi.launch 是否已启动）"
        )
        return False

    def _lifecycle_set(self, node: str, transition: str, timeout: float) -> bool:
        if not self._node_available(node):
            self.get_logger().warn(f"/{node} 不存在，跳过 lifecycle {transition}")
            return False
        cmd = ["ros2", "lifecycle", "set", f"/{node}", transition]
        self.get_logger().info(f"ros2 lifecycle set /{node} {transition}")
        try:
            proc = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=timeout,
                env=os.environ.copy(),
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error(f"/{node} {transition}: timeout {timeout:.0f}s")
            return False
        out = ((proc.stdout or "") + (proc.stderr or "")).strip()
        if proc.returncode != 0:
            self.get_logger().error(f"/{node} {transition} failed: {out}")
            return False
        self.get_logger().info(f"/{node} {transition} ok")
        return True

    def _ensure_lifecycle_active(
        self, node: str, configure_timeout: float, activate_timeout: float
    ) -> bool:
        """unconfigured→configure→activate；inactive→activate；active→成功。"""
        if not self._wait_for_node(node):
            return False

        state = self._lifecycle_state(node)
        if "active [3]" in state:
            return True
        if "inactive [2]" in state:
            return self._lifecycle_set(node, "activate", activate_timeout)
        if "unconfigured [1]" in state:
            if not self._lifecycle_set(node, "configure", configure_timeout):
                return False
            return self._lifecycle_set(node, "activate", activate_timeout)
        if "configuring" in state or "activating" in state:
            self.get_logger().info(f"/{node} 过渡中 ({state})，下次重试")
            return False

        self.get_logger().warn(f"/{node} 未知 lifecycle 状态: {state!r}，等待重试")
        return False

    def _activate_costmap(self) -> bool:
        if not self._ctrl_ok:
            self._publish("COSTMAP_CONFIGURING")
        if self._ensure_lifecycle_active(
            self._costmap_node, self._costmap_cfg_t, self._costmap_act_t
        ):
            self._costmap_ok = True
            self._costmap_pub.publish(Bool(data=True))
            if not self._ctrl_ok:
                self._publish("COSTMAP_ACTIVE")
            self.get_logger().info(f"{self._costmap_node} active")
            return True
        return False

    def _activate_controller(self) -> bool:
        self._publish("CONTROLLER_CONFIGURING")
        if self._ensure_lifecycle_active(
            self._ctrl_node, self._ctrl_cfg_t, self._ctrl_act_t
        ):
            self._ctrl_ok = True
            self._publish("ACTIVE")
            self.get_logger().info("controller_server ACTIVE — FollowPath 可用")
            return True
        return False

    def _bringup_loop(self) -> None:
        time.sleep(self._startup_delay)
        while rclpy.ok() and (not self._costmap_ok or not self._ctrl_ok):
            try:
                if not self._costmap_ok:
                    self._activate_costmap()
                if not self._ctrl_ok:
                    self._activate_controller()
            except Exception as exc:
                self.get_logger().error(f"bringup exception: {exc}")
            if not self._costmap_ok or not self._ctrl_ok:
                self.get_logger().warn(f"{self._retry_period:.0f}s 后重试")
                time.sleep(self._retry_period)


def main() -> None:
    rclpy.init()
    node = Nav2ControllerBringupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
