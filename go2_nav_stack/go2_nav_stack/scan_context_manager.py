#!/usr/bin/env python3
"""Scan Context：整楼单库 scan_context_db.json（§13–15 SingleMap）。"""

from __future__ import annotations

from pathlib import Path
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

from go2_nav_stack.scan_context_lib import (
    load_db,
    make_descriptor,
    yaw_shift_distance,
)


class ScanContextManager(Node):
    def __init__(self) -> None:
        super().__init__("scan_context_manager")
        self.declare_parameter("maps_root", "/home/allen/maps")
        self.declare_parameter("scan_context_db", "scan_context_db.json")
        self.declare_parameter("search_scope", "whole_building")
        self.declare_parameter("top_k", 10)
        self.declare_parameter("max_ring_distance", 0.35)
        self._cloud_buf: List[np.ndarray] = []
        self._max_frames = 30
        self._entries: list = []
        self._load_db()
        self.create_subscription(PointCloud2, "/cloud_registered", self._on_cloud, 10)
        self._result_pub = self.create_publisher(String, "/scan_context/last_query", 10)
        self.create_service(Trigger, "~/query", self._on_query)
        self.get_logger().info(
            f"scan_context_manager: single_db entries={len(self._entries)} "
            f"scope={self.get_parameter('search_scope').value}"
        )

    def _db_path(self) -> Path:
        root = Path(self.get_parameter("maps_root").value).expanduser()
        name = self.get_parameter("scan_context_db").value
        p = root / name
        if not p.is_file():
            from ament_index_python.packages import get_package_share_directory

            share = Path(get_package_share_directory("go2_nav_stack")) / "config" / name
            if share.is_file():
                return share
        return p

    def _load_db(self) -> None:
        self._entries = load_db(self._db_path())

    def _on_cloud(self, msg: PointCloud2) -> None:
        pts = []
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pts.append((p[0], p[1], p[2]))
        if pts:
            self._cloud_buf.append(np.array(pts, dtype=np.float32))
            if len(self._cloud_buf) > self._max_frames:
                self._cloud_buf.pop(0)

    def _aggregate_xy(self) -> np.ndarray:
        if not self._cloud_buf:
            return np.zeros((0, 2), dtype=np.float32)
        all_pts = np.vstack(self._cloud_buf)
        return all_pts[:, :2]

    def _on_query(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        xy = self._aggregate_xy()
        if xy.shape[0] < 50:
            res.success = False
            res.message = "not enough points, move lidar or wait"
            return res
        if not self._entries:
            res.success = False
            res.message = "scan_context_db empty; run build_scan_context_db"
            return res
        query_desc = make_descriptor(xy)
        top_k = int(self.get_parameter("top_k").value)
        max_d = float(self.get_parameter("max_ring_distance").value)
        candidates = []
        for e in self._entries:
            d, shift = yaw_shift_distance(query_desc, e["descriptor"])
            if d <= max_d:
                yaw_off = shift * (2.0 * np.pi / 60.0)
                candidates.append(
                    (d, e["x"], e["y"], e["yaw"] + yaw_off)
                )
        candidates.sort(key=lambda x: x[0])
        candidates = candidates[:top_k]
        if not candidates:
            res.success = False
            res.message = "no scan context match"
            return res
        lines = [
            f"x={c[1]:.2f} y={c[2]:.2f} yaw={c[3]:.2f} d={c[0]:.3f}"
            for c in candidates
        ]
        msg = "; ".join(lines)
        self._result_pub.publish(String(data=msg))
        res.success = True
        res.message = msg
        return res


def main() -> None:
    rclpy.init()
    node = ScanContextManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
