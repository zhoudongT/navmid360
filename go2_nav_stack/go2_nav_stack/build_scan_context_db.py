#!/usr/bin/env python3
"""从 PCD + 位姿生成整楼 scan_context_db.json（单地图，无楼层/楼梯字段）。"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from go2_nav_stack.scan_context_lib import load_db, make_descriptor, save_db


def read_pcd_xy(path: Path, max_pts: int = 50000) -> np.ndarray:
    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    pts = []
    for line in lines:
        if line.startswith("#") or not line or line[0].isalpha():
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
    parser.add_argument("--append", action="store_true", help="追加到已有数据库")
    args = parser.parse_args()

    pcd = Path(args.pcd).expanduser()
    xy = read_pcd_xy(pcd)
    desc = make_descriptor(xy)
    root = Path(args.maps_root).expanduser()
    out = root / args.db_name
    entries = load_db(out) if args.append and out.is_file() else []
    kf_id = args.keyframe_id.strip() or str(len(entries))
    entries.append(
        {
            "keyframe_id": kf_id,
            "x": args.x,
            "y": args.y,
            "yaw": args.yaw,
            "descriptor": desc,
        }
    )
    save_db(out, entries)
    print(f"saved {out} keyframe={kf_id} total={len(entries)} pts={xy.shape[0]}")


if __name__ == "__main__":
    main()
