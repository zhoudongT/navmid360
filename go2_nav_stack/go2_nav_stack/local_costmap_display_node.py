#!/usr/bin/env python3
"""转发局部 costmap 到 /local_costmap/map，供 RViz 显示（Fixed Frame=map）。"""

from __future__ import annotations

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class LocalCostmapDisplayNode(Node):
    def __init__(self) -> None:
        super().__init__("local_costmap_display")
        self.declare_parameter(
            "input_topics",
            [
                "/local_costmap/costmap",
                "/costmap/costmap",
            ],
        )
        self.declare_parameter("output_topic", "/local_costmap/map")

        topics = list(self.get_parameter("input_topics").value)
        self._out_topic = str(self.get_parameter("output_topic").value)
        self._active_source = ""

        costmap_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(OccupancyGrid, self._out_topic, costmap_qos)
        self._alive_pub = self.create_publisher(Bool, "/local_costmap/available", 10)
        self._got_data = False

        for i, topic in enumerate(topics):
            self.create_subscription(
                OccupancyGrid,
                str(topic),
                lambda msg, t=str(topic): self._on_costmap(msg, t),
                costmap_qos,
            )

        self._watchdog = self.create_timer(10.0, self._on_watchdog)
        self.get_logger().info(
            f"局部地图显示: {topics} -> {self._out_topic} (RViz: Fixed Frame=map)"
        )

    def _on_costmap(self, msg: OccupancyGrid, source: str) -> None:
        if not msg.data:
            return

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id or "map"
        out.info = msg.info
        out.data = list(msg.data)
        self._pub.publish(out)
        self._alive_pub.publish(Bool(data=True))

        if self._active_source != source:
            self._active_source = source
            self.get_logger().info(
                f"局部 costmap 来自 {source}: frame={out.header.frame_id} "
                f"{out.info.width}x{out.info.height} @ {out.info.resolution:.2f}m"
            )
        self._got_data = True

    def _on_watchdog(self) -> None:
        if self._got_data:
            return
        self.get_logger().warn(
            "未收到 costmap（独立层: ros2 lifecycle get /costmap/costmap；"
            " 或 controller 内嵌: ros2 lifecycle get /local_costmap/local_costmap）",
            throttle_duration_sec=15.0,
        )


def main() -> None:
    rclpy.init()
    node = LocalCostmapDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
