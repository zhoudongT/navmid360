"""轻量 Scan Context 描述子（单张全局地图，无楼层/楼梯标注）。"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import List, Tuple

import numpy as np

RING_NUM = 20
SECTOR_NUM = 60
MAX_RADIUS = 40.0
DEFAULT_LOCAL_RADIUS = 25.0


def map_xy_to_body_xy(
    xy_map: np.ndarray, cx: float, cy: float, cyaw: float
) -> np.ndarray:
    """将 map 系点云变换到 (cx,cy,cyaw) 处的机体坐标系（Scan Context 匹配用）。"""
    if xy_map.size == 0:
        return xy_map
    dx = xy_map[:, 0].astype(np.float64) - cx
    dy = xy_map[:, 1].astype(np.float64) - cy
    c = math.cos(cyaw)
    s = math.sin(cyaw)
    bx = c * dx + s * dy
    by = -s * dx + c * dy
    return np.column_stack([bx, by]).astype(np.float32)


def make_local_descriptor_from_map(
    xy_map: np.ndarray,
    cx: float,
    cy: float,
    cyaw: float,
    local_radius: float = DEFAULT_LOCAL_RADIUS,
) -> np.ndarray:
    """从全局 map PCD 裁剪局部 patch，生成与 Livox 机体扫描同系的描述子。"""
    if xy_map.size == 0:
        return make_descriptor(xy_map)
    dist = np.hypot(xy_map[:, 0] - cx, xy_map[:, 1] - cy)
    mask = dist <= local_radius
    if not np.any(mask):
        return make_descriptor(np.zeros((0, 2), dtype=np.float32))
    body_xy = map_xy_to_body_xy(xy_map[mask], cx, cy, cyaw)
    return make_descriptor(body_xy)


def _ring_index(r: np.ndarray) -> np.ndarray:
    ring = np.floor(RING_NUM * r / MAX_RADIUS).astype(np.int32)
    return np.clip(ring, 0, RING_NUM - 1)


def _sector_index(az: np.ndarray) -> np.ndarray:
    sector = np.floor((az + np.pi) / (2.0 * np.pi) * SECTOR_NUM).astype(np.int32)
    return np.clip(sector, 0, SECTOR_NUM - 1)


def make_descriptor(points_xy: np.ndarray) -> np.ndarray:
    if points_xy.size == 0:
        return np.zeros((RING_NUM, SECTOR_NUM), dtype=np.float32)
    x = points_xy[:, 0]
    y = points_xy[:, 1]
    r = np.hypot(x, y)
    az = np.arctan2(y, x)
    mask = (r > 1.0) & (r < MAX_RADIUS)
    if not np.any(mask):
        return np.zeros((RING_NUM, SECTOR_NUM), dtype=np.float32)
    r = r[mask]
    ring = _ring_index(r)
    sector = _sector_index(az)
    desc = np.zeros((RING_NUM, SECTOR_NUM), dtype=np.float32)
    for ri, si in zip(ring, sector):
        desc[ri, si] = max(desc[ri, si], 1.0)
    return desc


def ring_key(desc: np.ndarray) -> np.ndarray:
    return np.max(desc, axis=1)


def distance(a: np.ndarray, b: np.ndarray) -> float:
    ka = ring_key(a)
    kb = ring_key(b)
    diff = np.abs(ka - kb).sum()
    denom = np.maximum(ka, kb).sum() + 1e-6
    return float(diff / denom)


def yaw_shift_distance(a: np.ndarray, b: np.ndarray) -> Tuple[float, int]:
    best_d = 1e9
    best_shift = 0
    for shift in range(SECTOR_NUM):
        rolled = np.roll(b, shift, axis=1)
        d = distance(a, rolled)
        if d < best_d:
            best_d = d
            best_shift = shift
    return best_d, int(best_shift)


def save_db(path: Path, entries: List[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = []
    for e in entries:
        payload.append(
            {
                "keyframe_id": str(e.get("keyframe_id", "0")),
                "x": float(e["x"]),
                "y": float(e["y"]),
                "yaw": float(e["yaw"]),
                "descriptor": e["descriptor"].tolist(),
            }
        )
    path.write_text(json.dumps(payload), encoding="utf-8")


def load_db(path: Path) -> List[dict]:
    if not path.is_file():
        return []
    data = json.loads(path.read_text(encoding="utf-8"))
    out = []
    for item in data:
        out.append(
            {
                "keyframe_id": str(item.get("keyframe_id", "0")),
                "x": float(item["x"]),
                "y": float(item["y"]),
                "yaw": float(item["yaw"]),
                "descriptor": np.array(item["descriptor"], dtype=np.float32),
            }
        )
    return out
