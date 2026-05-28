#!/usr/bin/env python3
"""将 /planned_path 与 EGO 轨迹转为 RViz 易显示的 Path（保留原始 Z，含楼梯）。"""

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

try:
    from traj_utils.msg import Bspline
except ImportError:
    Bspline = None  # type: ignore[misc, assignment]

try:
    from visualization_msgs.msg import Marker
except ImportError:
    Marker = None  # type: ignore[misc, assignment]


def _densify_control_points(
    pts: list[tuple[float, float, float]], per_seg: int = 10
) -> list[tuple[float, float, float]]:
    if len(pts) < 2:
        return list(pts)
    out: list[tuple[float, float, float]] = []
    n_seg = max(2, per_seg)
    for i in range(len(pts) - 1):
        x0, y0, z0 = pts[i]
        x1, y1, z1 = pts[i + 1]
        for k in range(n_seg):
            a = k / n_seg
            out.append(
                (
                    x0 + a * (x1 - x0),
                    y0 + a * (y1 - y0),
                    z0 + a * (z1 - z0),
                )
            )
    out.append(pts[-1])
    return out


class EgoTrajDisplayNode(Node):
    def __init__(self) -> None:
        super().__init__("ego_traj_display")
        if PositionCommand is None:
            raise RuntimeError("需要 quadrotor_msgs（编译 ego-planner-swarm）")

        self.declare_parameter("pos_cmd_topic", "/drone_0_planning/pos_cmd")
        self.declare_parameter("bspline_topic", "/drone_0_planning/bspline")
        self.declare_parameter("optimal_list_topic", "/drone_0_plan_vis/optimal_list")
        self.declare_parameter("ego_path_topic", "/ego_exec_path")
        self.declare_parameter("local_path_topic", "/ego_local_path")
        self.declare_parameter("global_path_in", "/planned_path")
        self.declare_parameter("global_path_viz_out", "/planned_path_viz")
        self.declare_parameter("trail_max_points", 120)
        self.declare_parameter("local_path_segments", 10)
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
        self._local_seg = max(2, int(self.get_parameter("local_path_segments").value))
        self._trail: deque[tuple[float, float, float]] = deque(maxlen=self._trail_max)
        self._global_path: Path | None = None
        self._local_path: Path | None = None
        self._local_from_bspline = False

        self._ego_pub = self.create_publisher(
            Path, str(self.get_parameter("ego_path_topic").value), viz_qos
        )
        self._local_pub = self.create_publisher(
            Path, str(self.get_parameter("local_path_topic").value), viz_qos
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
        if Bspline is not None:
            self.create_subscription(
                Bspline,
                str(self.get_parameter("bspline_topic").value),
                self._on_bspline,
                10,
            )
        else:
            self.get_logger().warn(
                "未找到 traj_utils/Bspline，将尝试用 optimal_list Marker 填充 /ego_local_path"
            )

        if Marker is not None:
            self.create_subscription(
                Marker,
                str(self.get_parameter("optimal_list_topic").value),
                self._on_optimal_marker,
                10,
            )

        self.create_subscription(
            Path,
            str(self.get_parameter("global_path_in").value),
            self._on_planned_path,
            latched,
        )

        hz = max(0.5, float(self.get_parameter("republish_hz").value))
        self.create_timer(1.0 / hz, self._republish_cached)
        self.get_logger().info(
            "ego_traj_display: /planned_path -> /planned_path_viz; "
            "pos_cmd -> /ego_exec_path; bspline/optimal_list -> /ego_local_path（红）"
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

    def _publish_local_path(
        self, pts: list[tuple[float, float, float]], frame_id: str, from_bspline: bool
    ) -> None:
        if len(pts) < 2:
            return
        if not from_bspline and self._local_from_bspline and self._local_path is not None:
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id or "map"
        for x, y, z in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._local_path = path
        self._local_from_bspline = from_bspline
        self._local_pub.publish(path)

    def _on_bspline(self, msg: Bspline) -> None:
        if len(msg.pos_pts) < 2:
            return
        ctrl = [(float(p.x), float(p.y), float(p.z)) for p in msg.pos_pts]
        dense = _densify_control_points(ctrl, self._local_seg)
        self._publish_local_path(dense, "map", from_bspline=True)

    def _on_optimal_marker(self, msg: Marker) -> None:
        # 已有 bspline 轨迹时不再用 optimal_list，避免 RViz 上出现两条来回折的线
        if self._local_from_bspline:
            return
        if msg.type != Marker.LINE_STRIP or len(msg.points) < 2:
            return
        pts = [(float(p.x), float(p.y), float(p.z)) for p in msg.points]
        self._publish_local_path(pts, msg.header.frame_id, from_bspline=False)

    def _on_planned_path(self, msg: Path) -> None:
        if msg.poses:
            self._global_path = msg
            self._publish_global()

    def _republish_cached(self) -> None:
        if self._global_path is not None:
            self._publish_global()
        if self._local_path is not None and self._local_path.poses:
            out = Path()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self._local_path.header.frame_id or "map"
            out.poses = list(self._local_path.poses)
            self._local_pub.publish(out)

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
