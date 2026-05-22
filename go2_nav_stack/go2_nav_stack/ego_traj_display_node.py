#!/usr/bin/env python3
"""将 EGO / 全局路径转为 RViz 易显示的 Path（解决 TRANSIENT_LOCAL 与 RViz 默认 QoS 不匹配）。"""

from __future__ import annotations

from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

try:
    from quadrotor_msgs.msg import PositionCommand
except ImportError:
    PositionCommand = None  # type: ignore[misc, assignment]


class EgoTrajDisplayNode(Node):
    def __init__(self) -> None:
        super().__init__("ego_traj_display")
        if PositionCommand is None:
            raise RuntimeError("需要 quadrotor_msgs（编译 ego-planner-swarm）")

        self.declare_parameter("pos_cmd_topic", "/drone_0_planning/pos_cmd")
        self.declare_parameter("ego_path_topic", "/ego_exec_path")
        self.declare_parameter("global_path_in", "/planned_path")
        self.declare_parameter("global_path_viz_out", "/planned_path_viz")
        self.declare_parameter("trail_max_points", 80)
        self.declare_parameter("republish_hz", 2.0)

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        viz_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._trail_max = int(self.get_parameter("trail_max_points").value)
        self._trail: deque[tuple[float, float, float]] = deque(maxlen=self._trail_max)
        self._global_path: Path | None = None

        self._ego_pub = self.create_publisher(
            Path, str(self.get_parameter("ego_path_topic").value), viz_qos
        )
        self._global_pub = self.create_publisher(
            Path, str(self.get_parameter("global_path_viz_out").value), viz_qos
        )

        self.create_subscription(
            PositionCommand,
            str(self.get_parameter("pos_cmd_topic").value),
            self._on_pos_cmd,
            20,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("global_path_in").value),
            self._on_global,
            latched,
        )

        hz = max(0.5, float(self.get_parameter("republish_hz").value))
        self.create_timer(1.0 / hz, self._republish_global)
        self.get_logger().info(
            "ego_traj_display: /planned_path -> /planned_path_viz, "
            "pos_cmd -> /ego_exec_path（RViz Path 请订阅带 _viz 的话题）"
        )

    def _on_pos_cmd(self, msg: PositionCommand) -> None:
        self._trail.append(
            (float(msg.position.x), float(msg.position.y), float(msg.position.z))
        )
        if len(self._trail) < 2:
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for x, y, z in self._trail:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._ego_pub.publish(path)

    def _on_global(self, msg: Path) -> None:
        if msg.poses:
            self._global_path = msg
            self._publish_global()

    def _republish_global(self) -> None:
        if self._global_path is not None:
            self._publish_global()

    def _publish_global(self) -> None:
        if self._global_path is None or not self._global_path.poses:
            return
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._global_path.header.frame_id or "map"
        out.poses = list(self._global_path.poses)
        self._global_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = EgoTrajDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
