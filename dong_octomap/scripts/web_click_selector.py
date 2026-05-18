#!/usr/bin/env python3

import math
from typing import Optional, Set, Tuple

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


GridIndex = Tuple[int, int, int]


class WebClickSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("web_click_selector")

        self.declare_parameter("occupied_marker_topic", "/octomap_occupied_markers")
        self.declare_parameter("preblocked_marker_topic", "/preblocked_cells_markers")
        self.declare_parameter("raw_click_topic", "/web_clicked_point")
        self.declare_parameter("marker_topic", "/selection_markers")
        self.declare_parameter("start_topic", "/start_point")
        self.declare_parameter("goal_topic", "/goal_point")
        self.declare_parameter("status_topic", "/web_selection_status")
        self.declare_parameter("arrow_height", 0.6)
        self.declare_parameter("arrow_length", 0.7)
        self.declare_parameter("shaft_diameter", 0.16)
        self.declare_parameter("head_diameter", 0.32)
        self.declare_parameter("head_length", 0.44)
        self.declare_parameter("cube_size", 0.20)
        self.declare_parameter("robot_radius", 0.20)
        self.declare_parameter("snap_search_radius_cells", 13)
        self.declare_parameter("require_ground_support", True)
        self.declare_parameter("strict_direct_ground_support", False)
        self.declare_parameter("ground_support_xy_radius_cells", 1)
        self.declare_parameter("ground_support_depth_cells", 2)

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        occupied_marker_topic = self.get_parameter("occupied_marker_topic").value
        preblocked_marker_topic = self.get_parameter("preblocked_marker_topic").value
        raw_click_topic = self.get_parameter("raw_click_topic").value
        marker_topic = self.get_parameter("marker_topic").value
        start_topic = self.get_parameter("start_topic").value
        goal_topic = self.get_parameter("goal_topic").value
        status_topic = self.get_parameter("status_topic").value

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, transient_qos)
        self.start_pub = self.create_publisher(PointStamped, start_topic, transient_qos)
        self.goal_pub = self.create_publisher(PointStamped, goal_topic, transient_qos)
        self.status_pub = self.create_publisher(String, status_topic, transient_qos)

        self.create_subscription(Marker, occupied_marker_topic, self._on_occupied, transient_qos)
        self.create_subscription(Marker, preblocked_marker_topic, self._on_preblocked, transient_qos)
        self.create_subscription(PointStamped, raw_click_topic, self._on_raw_click, 10)

        self.expect_start = True
        self.has_start = False
        self.has_goal = False
        self.start_point: Optional[PointStamped] = None
        self.goal_point: Optional[PointStamped] = None

        self.map_frame = "map"
        self.resolution = 0.2
        self.occupied_cells: Set[GridIndex] = set()
        self.preblocked_cells: Set[GridIndex] = set()
        self.min_idx: Optional[GridIndex] = None
        self.max_idx: Optional[GridIndex] = None

        self._publish_status("等待占据栅格地图。")
        self.get_logger().info(
            f"web_click_selector started. raw_click_topic={raw_click_topic} occupied_marker_topic={occupied_marker_topic}"
        )

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _on_occupied(self, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST:
            return

        self.map_frame = msg.header.frame_id or self.map_frame
        self.resolution = max(1e-6, float(msg.scale.x))
        occupied: Set[GridIndex] = set()

        min_x = min_y = min_z = math.inf
        max_x = max_y = max_z = -math.inf

        for p in msg.points:
            idx = self._world_to_grid(p.x, p.y, p.z)
            occupied.add(idx)
            min_x = min(min_x, idx[0])
            min_y = min(min_y, idx[1])
            min_z = min(min_z, idx[2])
            max_x = max(max_x, idx[0])
            max_y = max(max_y, idx[1])
            max_z = max(max_z, idx[2])

        self.occupied_cells = occupied
        if occupied:
            self.min_idx = (int(min_x), int(min_y), int(min_z))
            self.max_idx = (int(max_x), int(max_y), int(max_z))
            self._publish_status(
                f"地图已就绪：{len(self.occupied_cells)} 个占据栅格。"
            )

    def _on_preblocked(self, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST:
            return

        preblocked: Set[GridIndex] = set()
        for p in msg.points:
            preblocked.add(self._world_to_grid(p.x, p.y, p.z))
        self.preblocked_cells = preblocked

    def _on_raw_click(self, msg: PointStamped) -> None:
        if not self.occupied_cells or self.min_idx is None or self.max_idx is None:
            self._publish_status("地图尚未就绪。")
            return

        # 直通原始选点，与 map_viewer_gui 一致；吸附与可通行判定全部由 dong_path_node 完成。
        snapped_msg = PointStamped()
        snapped_msg.header = msg.header
        if not snapped_msg.header.frame_id:
            snapped_msg.header.frame_id = self.map_frame
        snapped_msg.header.stamp = self.get_clock().now().to_msg()
        snapped_msg.point = msg.point

        px, py, pz = msg.point.x, msg.point.y, msg.point.z

        if self.expect_start:
            self.start_point = snapped_msg
            self.has_start = True
            self.start_pub.publish(snapped_msg)
            self._publish_status(
                f"起点已发送 [{px:.2f}, {py:.2f}, {pz:.2f}]（由规划节点吸附到可通行格）。再次点击设置终点。"
            )
            self.get_logger().info(
                f"Set START raw click: [{px:.3f}, {py:.3f}, {pz:.3f}] frame={snapped_msg.header.frame_id}"
            )
        else:
            self.goal_point = snapped_msg
            self.has_goal = True
            self.goal_pub.publish(snapped_msg)
            self._publish_status(
                f"终点已发送 [{px:.2f}, {py:.2f}, {pz:.2f}]（由规划节点吸附）。再次点击设置起点。"
            )
            self.get_logger().info(
                f"Set GOAL raw click: [{px:.3f}, {py:.3f}, {pz:.3f}] frame={snapped_msg.header.frame_id}"
            )

        self.expect_start = not self.expect_start
        self._publish_markers()

    def _world_to_grid(self, x: float, y: float, z: float) -> GridIndex:
        r = self.resolution
        return (math.floor(x / r), math.floor(y / r), math.floor(z / r))

    def _make_arrow(self, marker_id: int, p: PointStamped, rgb: Tuple[float, float, float]) -> Marker:
        marker = Marker()
        marker.header = p.header
        marker.ns = "web_selector"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = float(self.get_parameter("shaft_diameter").value)
        marker.scale.y = float(self.get_parameter("head_diameter").value)
        marker.scale.z = float(self.get_parameter("head_length").value)
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0

        base = Point()
        base.x = p.point.x
        base.y = p.point.y
        base.z = p.point.z + float(self.get_parameter("arrow_height").value)
        tip = Point()
        tip.x = base.x
        tip.y = base.y
        tip.z = base.z - float(self.get_parameter("arrow_length").value)

        marker.points = [base, tip]
        return marker

    def _make_cube(self, marker_id: int, p: PointStamped, rgb: Tuple[float, float, float]) -> Marker:
        marker = Marker()
        marker.header = p.header
        marker.ns = "web_selector"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = p.point
        marker.pose.orientation.w = 1.0
        cube_size = float(self.get_parameter("cube_size").value)
        marker.scale.x = cube_size
        marker.scale.y = cube_size
        marker.scale.z = cube_size
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 0.95
        return marker

    def _publish_markers(self) -> None:
        markers = MarkerArray()
        if self.has_start and self.start_point is not None:
            markers.markers.append(self._make_arrow(0, self.start_point, (0.1, 0.95, 0.1)))
            markers.markers.append(self._make_cube(2, self.start_point, (0.1, 0.95, 0.1)))
        if self.has_goal and self.goal_point is not None:
            markers.markers.append(self._make_arrow(1, self.goal_point, (0.95, 0.1, 0.1)))
            markers.markers.append(self._make_cube(3, self.goal_point, (0.95, 0.1, 0.1)))
        self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebClickSelectorNode()
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
