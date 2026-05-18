#!/usr/bin/env python3
"""
冒烟：若已运行 map_package_manager + dong_path_node（及图层话题），调用 save_package 并检查写出 meta.yaml。
无服务时退出码 0（SKIP）；成功 0（OK）；失败 1。
"""
from __future__ import annotations

import os
import shutil
import sys
import tempfile

import rclpy
from dong_map_msgs.srv import SaveNavigationMapPackage
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


def main() -> None:
    rclpy.init()
    node = Node(f"map_save_smoke_{os.getpid()}")
    client = node.create_client(SaveNavigationMapPackage, "/map_package_manager/save_package")
    if not client.wait_for_service(timeout_sec=5.0):
        print("SKIP: /map_package_manager/save_package 不可用（未 source 工作空间或未起地图节点）。")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    pkg = tempfile.mkdtemp(prefix="dong_map_smoke_")
    try:
        req = SaveNavigationMapPackage.Request()
        req.package_path = pkg
        req.overwrite = True
        req.fast_save = True
        future = client.call_async(req)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin_until_future_complete(future, timeout_sec=30.0)
        finally:
            executor.remove_node(node)
            executor.shutdown()
        if not future.done():
            print("FAIL: 保存服务 30s 内无响应")
            sys.exit(1)
        result = future.result()
        if result is None or not result.success:
            print(f"FAIL: {getattr(result, 'message', None)}")
            sys.exit(1)
        meta = os.path.join(pkg, "meta.yaml")
        if not os.path.isfile(meta):
            print(f"FAIL: 未生成 {meta}")
            sys.exit(1)
        print(f"OK: {result.message} manifest={result.manifest_path}")
        sys.exit(0)
    finally:
        shutil.rmtree(pkg, ignore_errors=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
