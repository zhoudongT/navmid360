#!/usr/bin/env python3
"""Synthetic publishers to stress-test far_planner (mimics far_terrain_bridge + tf_manager)."""

from __future__ import annotations

import math
import random
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2


class FarCrashRepro(Node):
    def __init__(self) -> None:
        super().__init__("far_crash_repro")
        self.declare_parameter("reloc_jump", True)
        self._reloc_jump = bool(self.get_parameter("reloc_jump").value)
        self._odom_pub = self.create_publisher(Odometry, "/state_estimation", 10)
        self._terrain_ext_pub = self.create_publisher(PointCloud2, "/terrain_map_ext", 5)
        self._terrain_pub = self.create_publisher(PointCloud2, "/terrain_map", 5)
        self._scan_pub = self.create_publisher(PointCloud2, "/registered_scan", 5)
        self._t = 0.0
        self._phase = 0
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"far_crash_repro: reloc_jump={self._reloc_jump}, odom + ~10k terrain"
        )

    def _make_cloud(self, n_obs: int = 8000, n_free: int = 2500) -> PointCloud2:
        pts: list[tuple[float, float, float, float]] = []
        rng = random.Random(42)
        cx, cy = 2.61, 0.20
        for _ in range(n_obs):
            ang = rng.uniform(0, 2 * math.pi)
            r = rng.uniform(0.5, 11.0)
            x = cx + r * math.cos(ang)
            y = cy + r * math.sin(ang)
            z = rng.uniform(0.0, 1.0)
            pts.append((x, y, z, 2.0))
        for _ in range(n_free):
            ang = rng.uniform(0, 2 * math.pi)
            r = rng.uniform(0.3, 8.0)
            x = cx + r * math.cos(ang)
            y = cy + r * math.sin(ang)
            z = rng.uniform(-0.05, 0.12)
            pts.append((x, y, z, 0.05))
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        return point_cloud2.create_cloud(
            header,
            [
                point_cloud2.PointField(
                    name="x", offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1
                ),
                point_cloud2.PointField(
                    name="y", offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1
                ),
                point_cloud2.PointField(
                    name="z", offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1
                ),
                point_cloud2.PointField(
                    name="intensity",
                    offset=12,
                    datatype=point_cloud2.PointField.FLOAT32,
                    count=1,
                ),
            ],
            pts,
        )

    def _tick(self) -> None:
        self._t += 0.1
        if self._reloc_jump and self._t >= 1.0 and self._phase == 0:
            self._phase = 1
            self.get_logger().warn("reloc jump: odom (0,0) -> (2.61, 0.20)")
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        if self._reloc_jump and self._phase == 0:
            ox, oy, oz = 0.0, 0.0, 0.0
        else:
            ox = 2.61 + 0.01 * math.sin(self._t)
            oy = 0.20 + 0.01 * math.cos(self._t)
            oz = 0.40
        odom.pose.pose.position.x = ox
        odom.pose.pose.position.y = oy
        odom.pose.pose.position.z = oz
        yaw = 0.33
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        self._odom_pub.publish(odom)

        cloud = self._make_cloud()
        self._terrain_ext_pub.publish(cloud)
        self._terrain_pub.publish(cloud)
        self._scan_pub.publish(cloud)


def main() -> None:
    rclpy.init(args=sys.argv)
    node = FarCrashRepro()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
