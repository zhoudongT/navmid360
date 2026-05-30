#!/usr/bin/env python3
"""将 Go2 导航目标桥接到 FAR Planner 的 /goal_point。"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool


class FarGoalBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("far_goal_bridge")
        self.declare_parameter("goal_in_topic", "/move_base_simple/goal")
        self.declare_parameter("far_goal_topic", "/goal_point")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("resume_on_start", True)

        self._far_goal_topic = str(self.get_parameter("far_goal_topic").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._resume = bool(self.get_parameter("resume_on_start").value)

        self._goal_pub = self.create_publisher(PointStamped, self._far_goal_topic, 10)
        self._update_pub = self.create_publisher(Bool, "/update_visibility_graph", 5)

        goal_in = str(self.get_parameter("goal_in_topic").value)
        self.create_subscription(PoseStamped, goal_in, self._on_goal, 10)
        self.create_subscription(Bool, "/start_navigation", self._on_start, 10)
        self.get_logger().info(f"far_goal_bridge: {goal_in} -> {self._far_goal_topic}")

    def _publish_goal(self, msg: PoseStamped) -> None:
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id or self._map_frame
        out.point = msg.pose.position
        self._goal_pub.publish(out)
        upd = Bool()
        upd.data = True
        self._update_pub.publish(upd)
        self.get_logger().info(
            f"FAR goal ({out.point.x:.2f}, {out.point.y:.2f}, {out.point.z:.2f})"
        )

    def _on_goal(self, msg: PoseStamped) -> None:
        self._publish_goal(msg)

    def _on_start(self, msg: Bool) -> None:
        if msg.data and self._resume:
            upd = Bool()
            upd.data = True
            self._update_pub.publish(upd)


def main() -> None:
    rclpy.init()
    node = FarGoalBridgeNode()
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
