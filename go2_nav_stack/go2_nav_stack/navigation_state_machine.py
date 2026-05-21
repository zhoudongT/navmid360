#!/usr/bin/env python3
"""导航状态机：TF 丢失时急停；局部由 Nav2 MPPI 负责避障。"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class NavigationStateMachine(Node):
    def __init__(self) -> None:
        super().__init__("navigation_state_machine")
        self._state = "IDLE"
        self._tf_ok = False
        self._reloc_ok = True
        self._planner_status = "CLEAR"
        self._state_pub = self.create_publisher(String, "/navigation/state", 10)
        self._estop_pub = self.create_publisher(Bool, "/navigation/emergency_stop", 10)
        self.create_subscription(Bool, "/navigation/tf_ok", self._on_tf, 10)
        self.create_subscription(Bool, "/relocalization/ok", self._on_reloc, 10)
        self.create_subscription(String, "/local_planner/status", self._on_planner, 10)
        self.create_subscription(Bool, "/start_navigation", self._on_start_nav, 10)
        self.create_timer(0.2, self._tick)
        self.get_logger().info("navigation_state_machine started")

    def _on_tf(self, msg: Bool) -> None:
        self._tf_ok = msg.data

    def _on_reloc(self, msg: Bool) -> None:
        self._reloc_ok = msg.data

    def _on_planner(self, msg: String) -> None:
        self._planner_status = msg.data.strip() or "CLEAR"

    def _on_start_nav(self, msg: Bool) -> None:
        if msg.data and self._state == "IDLE":
            self._state = "GLOBAL_PLAN_READY"

    def _tick(self) -> None:
        ps = self._planner_status
        if not self._tf_ok:
            self._state = "TF_LOST"
        elif ps in ("ABORTED", "REJECTED"):
            self._state = "MPPI_FAILED"
        elif ps == "GOAL_REACHED":
            self._state = "GOAL_REACHED"
        elif ps in (
            "MPPI_FOLLOWING",
            "PATH_READY",
            "ESCAPE_BACKUP",
            "ESCAPE_ROTATE",
        ):
            self._state = "LOCAL_NAVIGATION"
        elif ps == "ESCAPE_FAILED":
            self._state = "MPPI_FAILED"
        elif not self._reloc_ok:
            self._state = "RELOCALIZATION"
        elif self._state == "GLOBAL_PLAN_READY":
            self._state = "LOCAL_NAVIGATION"
        elif self._state == "IDLE":
            pass
        else:
            self._state = "LOCAL_NAVIGATION"

        estop = self._state == "TF_LOST"
        self._state_pub.publish(String(data=self._state))
        self._estop_pub.publish(Bool(data=estop))


def main() -> None:
    rclpy.init()
    node = NavigationStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
