#!/usr/bin/env python3
"""体素降采样 PCD，用于缩小先验图体积、加快 ICP/FAST-LIO 加载。

示例:
  python3 scripts/downsample_pcd.py \\
    /home/allen/maps/scans.pcd \\
    /home/allen/maps/scans_0.5m.pcd \\
    --leaf 0.5

  # 更稀疏（大场景导航常用 0.8~1.0 m）
  python3 scripts/downsample_pcd.py in.pcd out.pcd --leaf 1.0
"""

from __future__ import annotations

import argparse
import os
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description="Voxel downsample a PCD file (Open3D).")
    parser.add_argument("input", help="输入 .pcd 路径")
    parser.add_argument("output", help="输出 .pcd 路径")
    parser.add_argument(
        "--leaf",
        type=float,
        default=0.5,
        help="体素边长 (m)，越大点越少。默认 0.5",
    )
    parser.add_argument(
        "--estimate-normals",
        action="store_true",
        help="保留法向时先估计（输入含 normal 且需保留时再用，较慢）",
    )
    args = parser.parse_args()

    if args.leaf <= 0:
        print("error: --leaf 必须 > 0", file=sys.stderr)
        return 1
    if not os.path.isfile(args.input):
        print(f"error: 文件不存在: {args.input}", file=sys.stderr)
        return 1

    try:
        import open3d as o3d
    except ImportError:
        print("error: 需要 open3d: pip3 install open3d", file=sys.stderr)
        return 1

    print(f"读取: {args.input}")
    pcd = o3d.io.read_point_cloud(args.input)
    n_in = len(pcd.points)
    if n_in == 0:
        print("error: 点云为空", file=sys.stderr)
        return 1
    print(f"  原始点数: {n_in:,}")

    if args.estimate_normals and not pcd.has_normals():
        print("估计法向（可选，较慢）...")
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=args.leaf * 4, max_nn=30)
        )

    print(f"体素降采样 leaf={args.leaf} m ...")
    ds = pcd.voxel_down_sample(args.leaf)
    n_out = len(ds.points)
    ratio = 100.0 * n_out / n_in if n_in else 0.0
    print(f"  降采样后: {n_out:,} 点 ({ratio:.1f}%)")

    os.makedirs(os.path.dirname(os.path.abspath(args.output)) or ".", exist_ok=True)
    ok = o3d.io.write_point_cloud(args.output, ds, write_ascii=False)
    if not ok:
        print(f"error: 写入失败: {args.output}", file=sys.stderr)
        return 1

    out_mb = os.path.getsize(args.output) / (1024 * 1024)
    print(f"已保存: {args.output} ({out_mb:.1f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
