#!/usr/bin/env python3

from __future__ import annotations

import copy
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import yaml
from dong_map_msgs.srv import (
    ExportNavigationSnapshot,
    GetNavigationMapMeta,
    LoadNavigationMapPackage,
    SaveNavigationMapPackage,
)
from octomap_msgs.msg import Octomap
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker

from dong_octomap.ros_client_timeouts import (
    export_navigation_snapshot_wait_timeout_sec,
    get_navigation_map_meta_wait_timeout_sec,
)


class MapPackageManager(Node):
    def __init__(self) -> None:
        super().__init__("map_package_manager")

        self.declare_parameter("octomap_topic", "/octomap")
        self.declare_parameter("occupied_marker_topic", "/octomap_occupied_markers")
        self.declare_parameter("preblocked_topic", "/preblocked_cells_markers")
        self.declare_parameter("traversable_topic", "/traversable_cells_markers")
        self.declare_parameter("risk_cost_topic", "/risk_cost_cells")
        self.declare_parameter("planner_meta_service", "/dong_path_node/get_meta")
        self.declare_parameter("planner_export_service", "/dong_path_node/export_snapshot")
        self.declare_parameter("autoload_package_path", "")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        callback_group = ReentrantCallbackGroup()

        self._latest_octomap: Optional[Octomap] = None
        self._latest_occupied: Optional[Marker] = None
        self._latest_preblocked: Optional[Marker] = None
        self._latest_traversable: Optional[Marker] = None
        self._latest_risk_cost: Optional[PointCloud2] = None

        self.create_subscription(
            Octomap,
            self.get_parameter("octomap_topic").value,
            self._on_octomap,
            qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            Marker,
            self.get_parameter("occupied_marker_topic").value,
            self._on_occupied,
            qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            Marker,
            self.get_parameter("preblocked_topic").value,
            self._on_preblocked,
            qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            Marker,
            self.get_parameter("traversable_topic").value,
            self._on_traversable,
            qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("risk_cost_topic").value,
            self._on_risk_cost,
            qos,
            callback_group=callback_group,
        )

        self.octomap_pub = self.create_publisher(Octomap, self.get_parameter("octomap_topic").value, qos)
        self.occupied_pub = self.create_publisher(
            Marker, self.get_parameter("occupied_marker_topic").value, qos
        )
        self.preblocked_pub = self.create_publisher(
            Marker, self.get_parameter("preblocked_topic").value, qos
        )
        self.traversable_pub = self.create_publisher(
            Marker, self.get_parameter("traversable_topic").value, qos
        )
        self.risk_cost_pub = self.create_publisher(
            PointCloud2, self.get_parameter("risk_cost_topic").value, qos
        )

        self.meta_client = self.create_client(
            GetNavigationMapMeta,
            self.get_parameter("planner_meta_service").value,
            callback_group=callback_group,
        )
        self.export_client = self.create_client(
            ExportNavigationSnapshot,
            self.get_parameter("planner_export_service").value,
            callback_group=callback_group,
        )

        self.create_service(
            SaveNavigationMapPackage,
            "~/save_package",
            self._handle_save_package,
            callback_group=callback_group,
        )
        self.create_service(
            LoadNavigationMapPackage,
            "~/load_package",
            self._handle_load_package,
            callback_group=callback_group,
        )
        # 尽快加载，避免网页长时间显示「尚未加载栅格」
        self._autoload_timer = self.create_timer(0.3, self._autoload_package_once)

        self.get_logger().info(
            "map_package_manager started. save_service=~/save_package load_service=~/load_package"
        )

    def _on_octomap(self, msg: Octomap) -> None:
        self._latest_octomap = copy.deepcopy(msg)

    def _on_occupied(self, msg: Marker) -> None:
        if msg.type == Marker.CUBE_LIST:
            self._latest_occupied = copy.deepcopy(msg)

    def _on_preblocked(self, msg: Marker) -> None:
        if msg.type == Marker.CUBE_LIST:
            self._latest_preblocked = copy.deepcopy(msg)

    def _on_traversable(self, msg: Marker) -> None:
        if msg.type == Marker.CUBE_LIST:
            self._latest_traversable = copy.deepcopy(msg)

    def _on_risk_cost(self, msg: PointCloud2) -> None:
        self._latest_risk_cost = copy.deepcopy(msg)

    def _marker_points_to_numpy(self, marker: Marker) -> np.ndarray:
        return np.array([[p.x, p.y, p.z] for p in marker.points], dtype=np.float32)

    def _wait_for_future(
        self,
        future,
        timeout_sec: float | None,
        progress_stage: str | None = None,
    ):
        start = time.monotonic()
        last_log = start
        if progress_stage:
            self.get_logger().info(f"save_package 进度: {progress_stage} …")

        def _maybe_log_progress() -> None:
            nonlocal last_log
            if not progress_stage:
                return
            now = time.monotonic()
            if now - last_log >= 1.0:
                self.get_logger().info(
                    f"save_package 进度: {progress_stage}（已等待 {now - start:.0f}s）"
                )
                last_log = now

        if timeout_sec is None:
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self, timeout_sec=0.05)
                _maybe_log_progress()
        else:
            deadline = time.monotonic() + timeout_sec
            while rclpy.ok() and not future.done():
                if time.monotonic() > deadline:
                    return None
                rclpy.spin_once(self, timeout_sec=0.05)
                _maybe_log_progress()
        if not future.done():
            return None
        return future.result()

    def _run_with_save_progress_log(self, stage: str, fn):
        """在主线程执行 fn，另起守护线程每秒打一次进度（用于大块 npz 写入）。"""
        self.get_logger().info(f"save_package 进度: {stage} …")
        stop = threading.Event()
        start = time.monotonic()

        def _tick() -> None:
            while not stop.wait(1.0):
                self.get_logger().info(
                    f"save_package 进度: {stage}（已等待 {time.monotonic() - start:.0f}s）"
                )

        t = threading.Thread(target=_tick, daemon=True)
        t.start()
        try:
            return fn()
        finally:
            stop.set()
            t.join(timeout=0.2)

    def _call_export_snapshot(
        self, recompute_layers: bool
    ) -> tuple[bool, str, Optional[ExportNavigationSnapshot.Response]]:
        if not self.export_client.wait_for_service(timeout_sec=1.0):
            return False, "planner export service unavailable", None
        request = ExportNavigationSnapshot.Request()
        request.recompute_layers = recompute_layers
        future = self.export_client.call_async(request)
        export_stage = (
            "dong_path_node export_snapshot（重算禁行/可通行等图层）"
            if recompute_layers
            else "dong_path_node export_snapshot（快速保存，不重算图层）"
        )
        result = self._wait_for_future(
            future, export_navigation_snapshot_wait_timeout_sec(), export_stage
        )
        if result is None:
            return False, "planner export service timed out", None
        return result.success, result.message, result

    def _call_get_meta(self) -> tuple[bool, str, Optional[GetNavigationMapMeta.Response]]:
        if not self.meta_client.wait_for_service(timeout_sec=1.0):
            return False, "planner meta service unavailable", None
        future = self.meta_client.call_async(GetNavigationMapMeta.Request())
        result = self._wait_for_future(
            future,
            get_navigation_map_meta_wait_timeout_sec(),
            "dong_path_node get_navigation_map_meta",
        )
        if result is None:
            return False, "planner meta service timed out", None
        return result.success, result.message, result

    def _handle_save_package(
        self,
        request: SaveNavigationMapPackage.Request,
        response: SaveNavigationMapPackage.Response,
    ) -> SaveNavigationMapPackage.Response:
        fast = bool(getattr(request, "fast_save", False))
        package_dir_str = str(Path(request.package_path).expanduser())
        self.get_logger().info(
            f"save_package 开始: path={package_dir_str} overwrite={bool(request.overwrite)} "
            f"fast_save={fast}"
        )

        export_ok, export_msg, export_result = self._call_export_snapshot(
            recompute_layers=not fast
        )
        if not export_ok or export_result is None:
            response.success = False
            response.message = export_msg
            return response

        self.get_logger().info(
            "save_package 进度: export_snapshot 已完成，刷新订阅以获取最新 OctoMap（约 0.4s）"
        )
        # dong_path_node republishes /octomap at end of export; spin so _latest_octomap is fresh
        # before writing npz (avoids stale graph from another publisher winning depth=1).
        settle_deadline = time.monotonic() + 0.4
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        meta_ok, meta_msg, meta = self._call_get_meta()
        if not meta_ok or meta is None:
            response.success = False
            response.message = meta_msg
            return response

        if self._latest_octomap is None:
            response.success = False
            response.message = "octomap message not received yet"
            return response
        if self._latest_preblocked is None:
            response.success = False
            response.message = "preblocked marker not received yet"
            return response
        if self._latest_traversable is None:
            response.success = False
            response.message = "traversable marker not received yet"
            return response
        if self._latest_risk_cost is None:
            response.success = False
            response.message = "risk cost cloud not received yet"
            return response

        self.get_logger().info(
            "save_package 进度: 缓存校验通过，准备写入地图包（octomap / layers / meta.yaml）"
        )

        package_dir = Path(request.package_path).expanduser()
        if package_dir.exists():
            if not request.overwrite:
                response.success = False
                response.message = f"package path already exists: {package_dir}"
                return response
        package_dir.mkdir(parents=True, exist_ok=True)

        octomap_file = package_dir / "octomap_msg.npz"
        layers_file = package_dir / "layers.npz"
        meta_file = package_dir / "meta.yaml"

        self._run_with_save_progress_log(
            f"写入 {octomap_file.name}",
            lambda: np.savez_compressed(
                octomap_file,
                binary=np.array([self._latest_octomap.binary], dtype=np.bool_),
                octomap_id=np.array([self._latest_octomap.id]),
                resolution=np.array([self._latest_octomap.resolution], dtype=np.float64),
                frame_id=np.array([self._latest_octomap.header.frame_id]),
                data=np.array(self._latest_octomap.data, dtype=np.int8),
            ),
        )

        preblocked_points = self._marker_points_to_numpy(self._latest_preblocked)
        traversable_points = self._marker_points_to_numpy(self._latest_traversable)
        risk_records = list(
            point_cloud2.read_points(
                self._latest_risk_cost,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=True,
            )
        )
        risk_points = np.array(
            [[row[0], row[1], row[2], row[3]] for row in risk_records],
            dtype=np.float32,
        )
        layers_payload: dict[str, object] = {
            "preblocked_points": preblocked_points,
            "preblocked_scale": np.array(
                [
                    self._latest_preblocked.scale.x,
                    self._latest_preblocked.scale.y,
                    self._latest_preblocked.scale.z,
                ],
                dtype=np.float64,
            ),
            "preblocked_frame_id": np.array([self._latest_preblocked.header.frame_id]),
            "traversable_points": traversable_points,
            "traversable_scale": np.array(
                [
                    self._latest_traversable.scale.x,
                    self._latest_traversable.scale.y,
                    self._latest_traversable.scale.z,
                ],
                dtype=np.float64,
            ),
            "traversable_frame_id": np.array([self._latest_traversable.header.frame_id]),
            "risk_points": risk_points[:, :3] if risk_points.size else np.empty((0, 3), dtype=np.float32),
            "risk_intensity": risk_points[:, 3] if risk_points.size else np.empty((0,), dtype=np.float32),
            "risk_frame_id": np.array([self._latest_risk_cost.header.frame_id]),
        }
        occupied_count = 0
        if self._latest_occupied is not None:
            occ_pts = self._marker_points_to_numpy(self._latest_occupied)
            occupied_count = int(occ_pts.shape[0])
            layers_payload["occupied_points"] = occ_pts
            layers_payload["occupied_scale"] = np.array(
                [
                    self._latest_occupied.scale.x,
                    self._latest_occupied.scale.y,
                    self._latest_occupied.scale.z,
                ],
                dtype=np.float64,
            )
            layers_payload["occupied_frame_id"] = np.array([self._latest_occupied.header.frame_id])
        self._run_with_save_progress_log(
            f"写入 {layers_file.name}",
            lambda: np.savez_compressed(layers_file, **layers_payload),
        )

        meta_yaml = {
            "map_id": meta.map_id,
            "frame_id": meta.frame_id,
            "resolution": meta.resolution,
            "octomap_file": octomap_file.name,
            "layers_file": layers_file.name,
            "source_world_file": meta.source_world_file,
            "snapshot_stamp": {
                "sec": int(export_result.snapshot_stamp.sec),
                "nanosec": int(export_result.snapshot_stamp.nanosec),
            },
            "bounds": {
                "min": [meta.min_bound.x, meta.min_bound.y, meta.min_bound.z],
                "max": [meta.max_bound.x, meta.max_bound.y, meta.max_bound.z],
            },
            "planner": {
                "robot_radius": meta.robot_radius,
                "snap_search_radius_cells": meta.snap_search_radius_cells,
                "require_ground_support": meta.require_ground_support,
                "strict_direct_ground_support": meta.strict_direct_ground_support,
                "ground_support_xy_radius_cells": meta.ground_support_xy_radius_cells,
                "ground_support_depth_cells": meta.ground_support_depth_cells,
                "enable_preblocked_costmap": meta.enable_preblocked_costmap,
                "preblocked_costmap_radius_cells": meta.preblocked_costmap_radius_cells,
                "preblocked_costmap_weight": meta.preblocked_costmap_weight,
            },
            "layers": {
                "occupied_count": occupied_count,
                "preblocked_count": int(preblocked_points.shape[0]),
                "traversable_count": int(traversable_points.shape[0]),
                "risk_cost_count": int(risk_points.shape[0]),
            },
        }

        self.get_logger().info(f"save_package 进度: 写入 {meta_file.name}")
        with meta_file.open("w", encoding="utf-8") as f:
            yaml.safe_dump(meta_yaml, f, sort_keys=False, allow_unicode=True)

        self.get_logger().info(f"save_package 完成: {package_dir}")
        response.success = True
        response.message = "map package saved (fast)" if fast else "map package saved"
        response.manifest_path = str(meta_file)
        return response

    def _handle_load_package(
        self,
        request: LoadNavigationMapPackage.Request,
        response: LoadNavigationMapPackage.Response,
    ) -> LoadNavigationMapPackage.Response:
        success, message, map_id = self._load_package(request.package_path)
        response.success = success
        response.message = message
        response.map_id = map_id
        return response

    def _autoload_package_once(self) -> None:
        self._autoload_timer.cancel()
        package_path = str(self.get_parameter("autoload_package_path").value).strip()
        if not package_path:
            return

        success, message, map_id = self._load_package(package_path)
        if success:
            self.get_logger().info(
                f"autoloaded map package: {package_path} map_id={map_id}"
            )
        else:
            self.get_logger().error(
                f"failed to autoload map package {package_path}: {message}"
            )

    def _load_package(self, package_path: str) -> tuple[bool, str, str]:
        package_dir = Path(package_path).expanduser()
        meta_file = package_dir / "meta.yaml"
        if not meta_file.exists():
            return False, f"meta file not found: {meta_file}", ""

        with meta_file.open("r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)

        layers_path = package_dir / meta["layers_file"]
        if not layers_path.exists():
            return False, f"layers file not found: {layers_path}", ""

        octomap_npz = np.load(package_dir / meta["octomap_file"], allow_pickle=False)

        octomap_msg = Octomap()
        octomap_msg.header.frame_id = str(octomap_npz["frame_id"][0])
        octomap_msg.header.stamp = self.get_clock().now().to_msg()
        octomap_msg.binary = bool(octomap_npz["binary"][0])
        octomap_msg.id = str(octomap_npz["octomap_id"][0])
        octomap_msg.resolution = float(octomap_npz["resolution"][0])
        octomap_msg.data = octomap_npz["data"].astype(np.int8).tolist()

        # 只发布 OctoMap。若再发 layers.npz 里的禁行/可通行/风险，会在 dong_path_node
        # 已根据 octree 重算并发布后再次盖写话题（TRANSIENT_LOCAL 上最后一条胜出），
        # 网页等订阅方会长期看到「旧快照」往往比当前规划规则更松 → 可行域偏大。
        self._latest_octomap = copy.deepcopy(octomap_msg)
        self._latest_occupied = None
        self._latest_preblocked = None
        self._latest_traversable = None
        self._latest_risk_cost = None
        self.octomap_pub.publish(octomap_msg)
        self.get_logger().info(
            "load_package: published /octomap only; preblocked/traversable/risk come from dong_path_node"
        )

        return True, "map package loaded", str(meta.get("map_id", ""))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapPackageManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
