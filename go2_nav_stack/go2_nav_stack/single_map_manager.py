#!/usr/bin/env python3
"""单张全局地图管理（building_global_map.pcd，不切 map frame）。"""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class SingleMapManager(Node):
    def __init__(self) -> None:
        super().__init__("single_map_manager")
        self.declare_parameter("maps_root", "/home/allen/maps")
        self.declare_parameter("global_map_pcd", "scans_0.5m.pcd")
        self.declare_parameter("scan_context_db", "scan_context_db.json")
        self.declare_parameter("octomap_package", "window_filtered_all")
        root = Path(self.get_parameter("maps_root").value).expanduser()
        self._pcd = root / self.get_parameter("global_map_pcd").value
        self._sc_db = root / self.get_parameter("scan_context_db").value
        self._octo = root / self.get_parameter("octomap_package").value
        self._info_pub = self.create_publisher(String, "/navigation/single_map_info", 10)
        self.create_service(Trigger, "~/get_map_paths", self._on_get_paths)
        self.create_timer(2.0, self._publish)
        self._publish()
        self.get_logger().info(
            f"single_map_manager: pcd={self._pcd} sc_db={self._sc_db} octo={self._octo}"
        )

    def _publish(self) -> None:
        msg = (
            f"mode=single_map pcd={self._pcd} scan_context_db={self._sc_db} "
            f"octomap_package={self._octo}"
        )
        self._info_pub.publish(String(data=msg))

    def _on_get_paths(self, _req, res):
        res.success = self._pcd.is_file()
        res.message = f"pcd={self._pcd} db={self._sc_db}"
        return res


def main() -> None:
    rclpy.init()
    node = SingleMapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
