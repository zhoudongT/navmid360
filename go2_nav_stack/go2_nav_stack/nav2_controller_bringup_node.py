#!/usr/bin/env python3
"""先激活独立 /costmap/costmap，再激活 controller_server（MPPI）。

lifecycle 服务在独立 Reentrant 回调组中同步调用，避免在 timer 回调里 spin 同一 executor 导致死锁。
"""

from __future__ import annotations

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool, String

# lifecycle_msgs/state.hpp PRIMARYState
_UNCONFIGURED = 1
_INACTIVE = 2
_ACTIVE = 3


class _LifecycleClient:
    def __init__(self, node: Node, target: str, cb_group) -> None:
        name = target.lstrip("/")
        self._node = node
        self._get_state = node.create_client(
            GetState, f"/{name}/get_state", callback_group=cb_group
        )
        self._change_state = node.create_client(
            ChangeState, f"/{name}/change_state", callback_group=cb_group
        )

    def _wait_service(self, client, timeout_sec: float) -> bool:
        return client.wait_for_service(timeout_sec=max(0.5, timeout_sec))

    def get_state_id(self, timeout_sec: float) -> int | None:
        if not self._wait_service(self._get_state, timeout_sec):
            return None
        req = GetState.Request()
        try:
            resp = self._get_state.call(req)
        except Exception as exc:
            self._node.get_logger().warn(f"get_state 调用失败: {exc}")
            return None
        if resp is None:
            return None
        return int(resp.current_state.id)

    def change_state(self, transition_id: int, timeout_sec: float) -> bool:
        if not self._wait_service(self._change_state, timeout_sec):
            return False
        req = ChangeState.Request()
        req.transition.id = transition_id
        try:
            resp = self._change_state.call(req)
        except Exception as exc:
            self._node.get_logger().warn(f"change_state 调用失败: {exc}")
            return False
        if resp is None:
            return False
        return bool(resp.success)


class Nav2ControllerBringupNode(Node):
    def __init__(self) -> None:
        super().__init__("nav2_controller_bringup")
        self.declare_parameter("costmap_node", "costmap/costmap")
        self.declare_parameter("costmap_node_alt", "costmap")
        self.declare_parameter("skip_standalone_costmap_after_retries", 3)
        self.declare_parameter("controller_node", "controller_server")
        self.declare_parameter("startup_delay_sec", 5.0)
        self.declare_parameter("node_wait_timeout_sec", 45.0)
        self.declare_parameter("costmap_configure_timeout_sec", 60.0)
        self.declare_parameter("costmap_activate_timeout_sec", 30.0)
        self.declare_parameter("configure_timeout_sec", 180.0)
        self.declare_parameter("activate_timeout_sec", 90.0)
        self.declare_parameter("retry_period_sec", 12.0)

        self._costmap_name = str(self.get_parameter("costmap_node").value).lstrip("/")
        self._costmap_alt = str(self.get_parameter("costmap_node_alt").value).lstrip("/")
        self._skip_costmap_retries = int(
            self.get_parameter("skip_standalone_costmap_after_retries").value
        )
        self._costmap_skipped = False
        self._ctrl_name = str(self.get_parameter("controller_node").value).lstrip("/")
        self._startup_delay = float(self.get_parameter("startup_delay_sec").value)
        self._costmap_cfg_t = float(self.get_parameter("costmap_configure_timeout_sec").value)
        self._costmap_act_t = float(self.get_parameter("costmap_activate_timeout_sec").value)
        self._ctrl_cfg_t = float(self.get_parameter("configure_timeout_sec").value)
        self._ctrl_act_t = float(self.get_parameter("activate_timeout_sec").value)
        self._retry_period = float(self.get_parameter("retry_period_sec").value)
        self._costmap_ok = False
        self._ctrl_ok = False
        self._started = False
        self._retry_count = 0
        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._lc_group = ReentrantCallbackGroup()

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
            f"bringup: {self._startup_delay:.0f}s 后激活 MPPI costmap /{self._costmap_name}，再 /{self._ctrl_name}"
        )
        self.create_timer(
            self._startup_delay, self._on_startup_timer, callback_group=self._timer_group
        )
        self.create_timer(
            self._retry_period, self._on_retry_timer, callback_group=self._timer_group
        )
        self.create_subscription(
            Bool,
            "/relocalization/ok",
            self._on_relocalization_ok,
            10,
            callback_group=self._timer_group,
        )

    def _on_relocalization_ok(self, msg: Bool) -> None:
        if not msg.data or self._costmap_ok:
            return
        self.get_logger().info("重定位完成，立即重试激活 costmap …")
        self._tick_bringup()

    def _publish(self, text: str) -> None:
        self._status_pub.publish(String(data=text))

    def _node_in_graph(self, target: str) -> bool:
        target = target.lstrip("/")
        if "/" in target:
            parent_ns, leaf = target.rsplit("/", 1)
            want_ns = "/" + parent_ns if not parent_ns.startswith("/") else parent_ns
            for name, ns in self.get_node_names_and_namespaces():
                if name == leaf and ns.rstrip("/") == want_ns.rstrip("/"):
                    return True
            return False
        for name, _ns in self.get_node_names_and_namespaces():
            if name == target:
                return True
        return False

    def _wait_for_node(self, target: str) -> bool:
        if self._node_in_graph(target):
            return True
        self.get_logger().info(f"等待 /{target} 出现在图中 …", throttle_duration_sec=20.0)
        return False

    def _ensure_active(
        self,
        target: str,
        configure_timeout: float,
        activate_timeout: float,
    ) -> bool:
        if not self._wait_for_node(target):
            return False

        lc = _LifecycleClient(self, target, self._lc_group)
        state = lc.get_state_id(min(20.0, configure_timeout))
        if state is None:
            self.get_logger().warn(
                f"/{target} get_state 无响应（节点未就绪？）",
                throttle_duration_sec=15.0,
            )
            return False
        if state == _ACTIVE:
            return True
        if state == _INACTIVE:
            ok = lc.change_state(Transition.TRANSITION_ACTIVATE, activate_timeout)
            if not ok:
                self.get_logger().error(f"/{target} activate 失败")
            return ok
        if state == _UNCONFIGURED:
            self.get_logger().info(f"/{target} configuring …")
            if not lc.change_state(Transition.TRANSITION_CONFIGURE, configure_timeout):
                self.get_logger().error(
                    f"/{target} configure 失败：需 /cloud_registered_body（imu_link）与 odom->base TF"
                )
                return False
            ok = lc.change_state(Transition.TRANSITION_ACTIVATE, activate_timeout)
            if ok:
                self.get_logger().info(f"/{target} activate 成功")
            else:
                self.get_logger().error(f"/{target} configure 后 activate 失败")
            return ok
        self.get_logger().warn(f"/{target} 非常规状态 id={state}，稍后重试")
        return False

    def _try_activate_costmap(self) -> bool:
        names: list[str] = []
        if self._costmap_name:
            names.append(self._costmap_name)
        if self._costmap_alt and self._costmap_alt not in names:
            names.append(self._costmap_alt)
        for name in names:
            if self._ensure_active(name, self._costmap_cfg_t, self._costmap_act_t):
                if name != self._costmap_name:
                    self.get_logger().info(f"独立 costmap 使用备用节点 /{name}")
                return True
        return False

    def _tick_bringup(self) -> None:
        if self._costmap_ok and self._ctrl_ok:
            return
        self._retry_count += 1
        try:
            if not self._costmap_ok:
                if not self._ctrl_ok:
                    self._publish("COSTMAP_CONFIGURING")
                if self._try_activate_costmap():
                    self._costmap_ok = True
                    self._costmap_pub.publish(Bool(data=True))
                    if not self._ctrl_ok:
                        self._publish("COSTMAP_ACTIVE")
                    self.get_logger().info(
                        f"独立 costmap ACTIVE — RViz 请订阅 /costmap/costmap"
                    )
                elif (
                    self._skip_costmap_retries > 0
                    and self._retry_count >= self._skip_costmap_retries
                ):
                    self._costmap_skipped = True
                    self._costmap_ok = True
                    self.get_logger().warn(
                        f"独立 /costmap/costmap 激活失败 {self._retry_count} 次，"
                        "跳过并启动 controller（MPPI 使用内嵌 costmap）"
                    )
            if not self._ctrl_ok:
                self._publish("CONTROLLER_CONFIGURING")
                if self._ensure_active(self._ctrl_name, self._ctrl_cfg_t, self._ctrl_act_t):
                    self._ctrl_ok = True
                    self._publish("ACTIVE")
                    self.get_logger().info("controller_server ACTIVE — FollowPath 可用")
        except Exception as exc:
            self.get_logger().error(f"bringup exception: {exc}")

    def _on_startup_timer(self) -> None:
        if self._started:
            return
        self._started = True
        self._tick_bringup()

    def _on_retry_timer(self) -> None:
        if self._costmap_ok and self._ctrl_ok:
            return
        if self._retry_count % 3 == 0:
            self.get_logger().info("重试激活 costmap / controller_server …")
        self._tick_bringup()


def main() -> None:
    rclpy.init()
    node = Nav2ControllerBringupNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
