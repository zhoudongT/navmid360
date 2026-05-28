#!/usr/bin/env python3
"""从 PCD + 位姿生成整楼 scan_context_db.json（单地图，无楼层/楼梯字段）。

描述子须与 Livox 机体坐标系一致：从全局 PCD 裁剪 keyframe 附近局部 patch，
再变换到该位姿的 body 系后计算 Scan Context（勿用整图 PCD 直接算描述子）。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from go2_nav_stack.scan_context_lib import (
    DEFAULT_LOCAL_RADIUS,
    load_db,
    make_local_descriptor_from_map,
    save_db,
)


def read_pcd_xy(path: Path, max_pts: int = 500000) -> np.ndarray:
    raw = path.read_bytes()
    offset = 0
    n_pts = 0
    data_mode = "ascii"
    while offset < len(raw):
        end = raw.find(b"\n", offset)
        if end < 0:
            break
        line = raw[offset:end].decode("ascii", errors="ignore").strip()
        if line.startswith("POINTS"):
            n_pts = int(line.split()[1])
        elif line.startswith("WIDTH") and n_pts == 0:
            n_pts = int(line.split()[1])
        elif line.startswith("DATA"):
            data_mode = line.split()[1].strip().lower()
            offset = end + 1
            break
        offset = end + 1

    if n_pts <= 0:
        return np.zeros((0, 2), dtype=np.float32)

    count = min(n_pts, max_pts)
    if data_mode == "binary":
        need = count * 12
        if offset + need > len(raw):
            count = max(0, (len(raw) - offset) // 12)
        block = np.frombuffer(raw, dtype=np.float32, count=count * 3, offset=offset)
        if block.size < 2:
            return np.zeros((0, 2), dtype=np.float32)
        return block.reshape(-1, 3)[:, :2].astype(np.float32, copy=False)

    pts = []
    text = raw[offset:].decode("utf-8", errors="ignore").splitlines()
    for line in text:
        if not line or line[0].isalpha():
            continue
        sp = line.split()
        if len(sp) >= 2:
            try:
                pts.append((float(sp[0]), float(sp[1])))
            except ValueError:
                pass
        if len(pts) >= max_pts:
            break
    return np.array(pts, dtype=np.float32) if pts else np.zeros((0, 2))


def grid_keyframe_poses(
    xy_map: np.ndarray,
    step: float,
    local_radius: float,
    min_pts: int,
) -> list[tuple[float, float]]:
    if xy_map.size == 0:
        return []
    xs = xy_map[:, 0]
    ys = xy_map[:, 1]
    x0 = float(np.floor(xs.min() / step) * step + step * 0.5)
    x1 = float(np.ceil(xs.max() / step) * step)
    y0 = float(np.floor(ys.min() / step) * step + step * 0.5)
    y1 = float(np.ceil(ys.max() / step) * step)
    out: list[tuple[float, float]] = []
    x = x0
    while x <= x1:
        y = y0
        while y <= y1:
            dist = np.hypot(xs - x, ys - y)
            if int(np.count_nonzero(dist <= local_radius)) >= min_pts:
                out.append((x, y))
            y += step
        x += step
    return out


def main() -> None:
    parser = argparse.ArgumentParser(
        description="生成 scan_context_db.json（全局重定位粗匹配，无需 floor/node 标注）"
    )
    parser.add_argument("--pcd", required=True, help="全局先验 PCD，如 scans_0.5m.pcd")
    parser.add_argument("--maps-root", default="/home/allen/maps")
    parser.add_argument("--db-name", default="scan_context_db.json")
    parser.add_argument("--keyframe-id", default="", help="留空则自动递增 0,1,2...")
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--local-radius", type=float, default=DEFAULT_LOCAL_RADIUS)
    parser.add_argument(
        "--grid-step",
        type=float,
        default=0.0,
        help=">0 时在 PCD 上按网格自动生成 keyframe（推荐 8~12m）",
    )
    parser.add_argument(
        "--grid-min-pts",
        type=int,
        default=400,
        help="网格 keyframe 局部半径内最少点数",
    )
    parser.add_argument("--append", action="store_true", help="追加到已有数据库")
    args = parser.parse_args()

    pcd = Path(args.pcd).expanduser()
    xy = read_pcd_xy(pcd)
    if xy.shape[0] == 0:
        raise SystemExit(f"PCD 无有效 xy 点: {pcd}")

    root = Path(args.maps_root).expanduser()
    out = root / args.db_name
    entries = load_db(out) if args.append and out.is_file() else []

    local_radius = float(args.local_radius)
    poses: list[tuple[float, float, float]] = []
    if args.grid_step > 0.0:
        for x, y in grid_keyframe_poses(
            xy, float(args.grid_step), local_radius, int(args.grid_min_pts)
        ):
            poses.append((x, y, 0.0))
        print(f"grid keyframes: step={args.grid_step}m count={len(poses)}")
    else:
        poses.append((args.x, args.y, args.yaw))

    for x, y, yaw in poses:
        desc = make_local_descriptor_from_map(xy, x, y, yaw, local_radius)
        kf_id = args.keyframe_id.strip() or str(len(entries))
        entries.append(
            {
                "keyframe_id": kf_id,
                "x": x,
                "y": y,
                "yaw": yaw,
                "descriptor": desc,
            }
        )
        if args.keyframe_id:
            break
        args.keyframe_id = ""

    save_db(out, entries)
    print(f"saved {out} total={len(entries)} pcd_pts={xy.shape[0]}")


if __name__ == "__main__":
    main()
