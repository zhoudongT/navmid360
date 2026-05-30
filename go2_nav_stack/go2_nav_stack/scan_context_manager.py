#!/usr/bin/env python3
"""Scan Context：整楼单库 scan_context_db.json（§13–15 SingleMap）。"""

from __future__ import annotations

from pathlib import Path as FilePath
from typing import List

import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Int32, String
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
        self.declare_parameter("search_scope", "path_corridor")
        self.declare_parameter("path_corridor_margin_m", 22.0)
        self.declare_parameter("path_search_backward_points", 50)
        self.declare_parameter("path_search_forward_points", 100)
        self.declare_parameter("planned_path_topic", "/planned_path")
        self.declare_parameter("path_progress_topic", "/path_progress")
        self.declare_parameter("top_k", 10)
        self.declare_parameter("max_ring_distance", 0.35)
        self.declare_parameter("lidar_topic", "/livox/lidar")
        self.declare_parameter("registered_cloud_topic", "/cloud_registered")
        self.declare_parameter("min_query_points", 50)
        self._cloud_buf: List[np.ndarray] = []
        self._max_frames = 30
        self._entries: list = []
        self._path_xy: np.ndarray | None = None
        self._path_progress = 0
        self._load_db()
        lidar_topic = str(self.get_parameter("lidar_topic").value)
        reg_topic = str(self.get_parameter("registered_cloud_topic").value)
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            NavPath,
            str(self.get_parameter("planned_path_topic").value),
            self._on_path,
            path_qos,
        )
        self.create_subscription(
            Int32,
            str(self.get_parameter("path_progress_topic").value),
            self._on_progress,
            10,
        )
        # ICP 锁定前 FAST-LIO 不发布 /cloud_registered，粗定位须用原始 Livox
        self.create_subscription(CustomMsg, lidar_topic, self._on_livox, 10)
        self.create_subscription(PointCloud2, reg_topic, self._on_cloud, 10)
        self._result_pub = self.create_publisher(String, "/scan_context/last_query", 10)
        self.create_service(Trigger, "~/query", self._on_query)
        self.get_logger().info(
            f"scan_context_manager: entries={len(self._entries)} "
            f"lidar={lidar_topic} registered={reg_topic}"
        )

    def _db_path(self) -> FilePath:
        root = FilePath(self.get_parameter("maps_root").value).expanduser()
        name = self.get_parameter("scan_context_db").value
        p = root / name
        if not p.is_file():
            from ament_index_python.packages import get_package_share_directory

            share = FilePath(get_package_share_directory("go2_nav_stack")) / "config" / name
            if share.is_file():
                return share
        return p

    def _load_db(self) -> None:
        self._entries = load_db(self._db_path())

    def _push_scan(self, pts: np.ndarray) -> None:
        if pts.size == 0:
            return
        self._cloud_buf.append(pts.astype(np.float32, copy=False))
        if len(self._cloud_buf) > self._max_frames:
            self._cloud_buf.pop(0)

    def _on_livox(self, msg: CustomMsg) -> None:
        n = int(msg.point_num)
        if n <= 0:
            return
        pts = np.array([(p.x, p.y, p.z) for p in msg.points[:n]], dtype=np.float32)
        self._push_scan(pts)

    def _on_cloud(self, msg: PointCloud2) -> None:
        pts = []
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pts.append((p[0], p[1], p[2]))
        if pts:
            self._push_scan(np.array(pts, dtype=np.float32))

    def _aggregate_xy(self) -> np.ndarray:
        if not self._cloud_buf:
            return np.zeros((0, 2), dtype=np.float32)
        all_pts = np.vstack(self._cloud_buf)
        return all_pts[:, :2]

    def _on_path(self, msg: NavPath) -> None:
        if not msg.poses:
            self._path_xy = None
            return
        pts = [
            (float(ps.pose.position.x), float(ps.pose.position.y))
            for ps in msg.poses
        ]
        self._path_xy = np.array(pts, dtype=np.float32)

    def _on_progress(self, msg: Int32) -> None:
        self._path_progress = int(msg.data)

    def _dist_to_path_m(self, x: float, y: float) -> float:
        if self._path_xy is None or self._path_xy.shape[0] == 0:
            return 0.0
        d = np.hypot(self._path_xy[:, 0] - x, self._path_xy[:, 1] - y)
        return float(np.min(d))

    def _entries_for_search(self) -> list:
        scope = str(self.get_parameter("search_scope").value).strip()
        if scope != "path_corridor" or self._path_xy is None or self._path_xy.shape[0] < 2:
            return self._entries
        margin = float(self.get_parameter("path_corridor_margin_m").value)
        back = int(self.get_parameter("path_search_backward_points").value)
        fwd = int(self.get_parameter("path_search_forward_points").value)
        lo = max(0, self._path_progress - back)
        hi = min(self._path_xy.shape[0], self._path_progress + fwd + 1)
        corridor_pts = self._path_xy[lo:hi]
        if corridor_pts.shape[0] == 0:
            return self._entries
        filtered = []
        for e in self._entries:
            ex, ey = float(e["x"]), float(e["y"])
            d = float(np.min(np.hypot(corridor_pts[:, 0] - ex, corridor_pts[:, 1] - ey)))
            if d <= margin:
                filtered.append(e)
        if filtered:
            self.get_logger().info(
                f"SC path_corridor: {len(filtered)}/{len(self._entries)} entries "
                f"(progress={self._path_progress}, margin={margin}m)",
                throttle_duration_sec=10.0,
            )
            return filtered
        self.get_logger().warn(
            "SC path_corridor 无候选，回退 whole_building",
            throttle_duration_sec=5.0,
        )
        return self._entries

    def _on_query(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        xy = self._aggregate_xy()
        min_pts = int(self.get_parameter("min_query_points").value)
        if xy.shape[0] < min_pts:
            res.success = False
            res.message = f"not enough points ({xy.shape[0]}<{min_pts}), wait for Livox"
            return res
        if not self._entries:
            res.success = False
            res.message = "scan_context_db empty; run build_scan_context_db"
            return res
        query_desc = make_descriptor(xy)
        top_k = int(self.get_parameter("top_k").value)
        max_d = float(self.get_parameter("max_ring_distance").value)
        search_entries = self._entries_for_search()
        candidates = []
        for e in search_entries:
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
            res.message = "no scan context match (db 描述子与当前扫描不匹配，请重建 scan_context_db)"
            return res
        best_d = candidates[0][0]
        if len(self._entries) < 3:
            self.get_logger().warn(
                f"scan_context_db 仅 {len(self._entries)} 条 keyframe，粗定位覆盖范围有限"
            )
        if best_d > max_d * 0.85:
            self.get_logger().warn(
                f"Scan Context 最佳匹配距离 {best_d:.3f} 偏大（阈值 {max_d}），"
                "ICP 可能失败，请检查 scan_context_db 或手动 /initialpose"
            )
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
