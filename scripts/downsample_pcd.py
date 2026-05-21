#!/usr/bin/env python3
"""体素降采样 PCD，用于缩小先验图体积、加快 ICP/FAST-LIO 加载。

优先使用 Open3D；未安装时自动用 NumPy 回退（仅需 python3-numpy）。

单文件:
  python3 scripts/downsample_pcd.py \\
    /home/allen/maps/scans.pcd \\
    /home/allen/maps/scans_0.5m.pcd \\
    --leaf 0.5

批量:
  python3 scripts/downsample_pcd.py --leaf 0.5 --batch \\
    /home/allen/maps/scans.pcd /home/allen/maps/scans_0.5m.pcd

默认 maps 目录:
  python3 scripts/downsample_pcd.py --default-maps --leaf 0.5

可选安装 Open3D（更快）: pip3 install open3d
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
from typing import Sequence

import numpy as np

_TYPE_MAP = {
    ("F", 4): np.float32,
    ("F", 8): np.float64,
    ("U", 1): np.uint8,
    ("U", 2): np.uint16,
    ("U", 4): np.uint32,
    ("I", 1): np.int8,
    ("I", 2): np.int16,
    ("I", 4): np.int32,
}


def _default_maps_dir() -> str:
    return os.path.expanduser("~/maps")


def _auto_output_path(input_path: str, leaf: float, in_place: bool) -> str:
    if in_place:
        return input_path
    base, ext = os.path.splitext(input_path)
    name = os.path.basename(base)
    if abs(leaf - 0.5) < 1e-6:
        if name.endswith("_0.5m"):
            return f"{base}_redownsampled{ext}"
        return os.path.join(os.path.dirname(input_path), f"{name}_0.5m{ext}")
    suffix = f"_voxel{leaf:g}".replace(".", "p")
    return f"{base}{suffix}{ext}"


def _parse_pcd_header(path: str) -> tuple[dict, int]:
    """返回 (header, data_offset)：binary 为字节偏移，ascii 为 0（由读逻辑跳过表头）。"""
    header: dict = {}
    data_offset = 0
    with open(path, "rb") as f:
        while True:
            raw = f.readline()
            if not raw:
                raise ValueError(f"PCD 头不完整: {path}")
            line = raw.decode("ascii", errors="ignore").strip()
            parts = line.split()
            if not parts:
                continue
            key = parts[0].upper()
            if key == "VERSION":
                header["version"] = parts[1]
            elif key == "FIELDS":
                header["fields"] = parts[1:]
            elif key == "SIZE":
                header["sizes"] = [int(x) for x in parts[1:]]
            elif key == "TYPE":
                header["types"] = parts[1:]
            elif key == "COUNT":
                header["counts"] = [int(x) for x in parts[1:]]
            elif key == "WIDTH":
                header["width"] = int(parts[1])
            elif key == "HEIGHT":
                header["height"] = int(parts[1])
            elif key == "VIEWPOINT":
                header["viewpoint"] = " ".join(parts[1:])
            elif key == "POINTS":
                header["points"] = int(parts[1])
            elif key == "DATA":
                header["data"] = parts[1].lower()
                if header["data"] == "binary":
                    data_offset = f.tell()
                break
        else:
            raise ValueError(f"PCD 缺少 DATA 行: {path}")
    for req in ("fields", "sizes", "types", "counts", "points", "data"):
        if req not in header:
            raise ValueError(f"PCD 头缺少 {req}: {path}")
    return header, data_offset


def _pcd_point_dtype(header: dict) -> np.dtype:
    names: list[str] = []
    formats: list = []
    for field, size, typ, count in zip(
        header["fields"], header["sizes"], header["types"], header["counts"]
    ):
        base = _TYPE_MAP.get((typ.upper(), size))
        if base is None:
            raise ValueError(f"不支持的 PCD 字段类型 {typ}{size}: {field}")
        if count == 1:
            names.append(field)
            formats.append(base)
        else:
            for i in range(count):
                names.append(f"{field}_{i}")
                formats.append(base)
    return np.dtype(list(zip(names, formats)))


def _read_pcd_xyz(path: str) -> np.ndarray:
    header, offset = _parse_pcd_header(path)
    if "x" not in header["fields"] or "y" not in header["fields"] or "z" not in header["fields"]:
        raise ValueError(f"PCD 须含 x y z 字段: {path}")
    dtype = _pcd_point_dtype(header)
    n = header["points"]
    if header["data"] == "binary":
        with open(path, "rb") as f:
            f.seek(offset)
            buf = f.read(dtype.itemsize * n)
        if len(buf) < dtype.itemsize * n:
            raise ValueError(f"PCD 数据长度不足: {path}")
        cloud = np.frombuffer(buf, dtype=dtype, count=n)
    elif header["data"] == "ascii":
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            past_data = False
            rows: list[list[str]] = []
            for line in f:
                stripped = line.strip()
                if not past_data:
                    if stripped.upper().startswith("DATA"):
                        past_data = True
                    continue
                if not stripped or stripped.startswith("#"):
                    continue
                rows.append(stripped.split())
                if len(rows) >= n:
                    break
        if len(rows) < n:
            raise ValueError(f"ASCII PCD 点数不足 {len(rows)} < {n}: {path}")
        xi = header["fields"].index("x")
        yi = header["fields"].index("y")
        zi = header["fields"].index("z")
        xyz = np.array(
            [[float(r[xi]), float(r[yi]), float(r[zi])] for r in rows[:n]],
            dtype=np.float64,
        )
        return xyz
    else:
        raise ValueError(f"仅支持 DATA ascii/binary，当前 {header['data']}: {path}")

    return np.column_stack(
        (cloud["x"].astype(np.float64), cloud["y"].astype(np.float64), cloud["z"].astype(np.float64))
    )


def _write_pcd_xyz_binary(path: str, xyz: np.ndarray) -> None:
    n = xyz.shape[0]
    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS x y z",
            "SIZE 4 4 4",
            "TYPE F F F",
            "COUNT 1 1 1",
            f"WIDTH {n}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {n}",
            "DATA binary",
            "",
        ]
    )
    pts = xyz.astype(np.float32, copy=False)
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(pts.tobytes())


def _voxel_downsample_numpy(xyz: np.ndarray, leaf: float) -> np.ndarray:
    vox = np.floor(xyz / leaf).astype(np.int64)
    packed = np.ascontiguousarray(vox).view(
        np.dtype((np.void, vox.dtype.itemsize * vox.shape[1]))
    )
    _, idx = np.unique(packed, return_index=True)
    idx.sort()
    return xyz[idx]


def _downsample_open3d(input_path: str, leaf: float, estimate_normals: bool):
    import open3d as o3d

    pcd = o3d.io.read_point_cloud(input_path)
    n_in = len(pcd.points)
    if n_in == 0:
        raise ValueError("点云为空")
    if estimate_normals and not pcd.has_normals():
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=leaf * 4, max_nn=30)
        )
    ds = pcd.voxel_down_sample(leaf)
    return np.asarray(ds.points), n_in


def _downsample_numpy(input_path: str, leaf: float, estimate_normals: bool):
    if estimate_normals:
        print("warning: --estimate-normals 仅 Open3D 模式支持，NumPy 回退已忽略", file=sys.stderr)
    xyz = _read_pcd_xyz(input_path)
    n_in = xyz.shape[0]
    if n_in == 0:
        raise ValueError("点云为空")
    out = _voxel_downsample_numpy(xyz, leaf)
    return out, n_in


def downsample_one(
    input_path: str,
    output_path: str,
    leaf: float,
    estimate_normals: bool,
) -> int:
    print(f"读取: {input_path}")
    backend = "open3d"
    try:
        xyz, n_in = _downsample_open3d(input_path, leaf, estimate_normals)
    except ImportError:
        backend = "numpy"
        try:
            xyz, n_in = _downsample_numpy(input_path, leaf, estimate_normals)
        except Exception as exc:
            print(f"error: {exc}", file=sys.stderr)
            print(
                "hint: 可安装 Open3D: unset http_proxy https_proxy; pip3 install open3d",
                file=sys.stderr,
            )
            return 1
    except Exception as exc:
        print(f"error (open3d): {exc}", file=sys.stderr)
        return 1

    n_out = xyz.shape[0]
    ratio = 100.0 * n_out / n_in if n_in else 0.0
    print(f"  后端: {backend}")
    print(f"  原始点数: {n_in:,}")
    print(f"  体素降采样 leaf={leaf} m ...")
    print(f"  降采样后: {n_out:,} 点 ({ratio:.1f}%)")

    out_dir = os.path.dirname(os.path.abspath(output_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    tmp_out = output_path
    if os.path.abspath(input_path) == os.path.abspath(output_path):
        tmp_out = output_path + ".tmp.pcd"

    try:
        if backend == "open3d":
            import open3d as o3d

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz)
            ok = o3d.io.write_point_cloud(tmp_out, pcd, write_ascii=False)
            if not ok:
                raise OSError(f"写入失败: {tmp_out}")
        else:
            _write_pcd_xyz_binary(tmp_out, xyz)
    except Exception as exc:
        print(f"error: 写入失败: {exc}", file=sys.stderr)
        return 1

    if tmp_out != output_path:
        shutil.move(tmp_out, output_path)

    out_mb = os.path.getsize(output_path) / (1024 * 1024)
    in_mb = os.path.getsize(input_path) / (1024 * 1024)
    print(f"  已保存: {output_path} ({out_mb:.1f} MB, 原文件 {in_mb:.1f} MB)\n")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Voxel downsample PCD (Open3D or NumPy fallback).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("input", nargs="?", help="输入 .pcd")
    parser.add_argument("output", nargs="?", help="输出 .pcd")
    parser.add_argument("--leaf", type=float, default=0.5, help="体素边长 (m)")
    parser.add_argument("--batch", nargs="+", metavar="PCD", help="批量输入")
    parser.add_argument("--default-maps", action="store_true", help="处理 ~/maps 下 scans*.pcd")
    parser.add_argument("--maps-dir", default=_default_maps_dir())
    parser.add_argument("--in-place", action="store_true")
    parser.add_argument("--backup", action="store_true")
    parser.add_argument("--estimate-normals", action="store_true")
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.leaf <= 0:
        print("error: --leaf 必须 > 0", file=sys.stderr)
        return 1

    inputs: list[str] = []
    outputs: list[str] = []

    if args.default_maps:
        maps_dir = os.path.expanduser(args.maps_dir)
        inputs = [
            os.path.join(maps_dir, "scans.pcd"),
            os.path.join(maps_dir, "scans_0.5m.pcd"),
        ]
        outputs = [
            os.path.join(maps_dir, "scans_0.5m.pcd"),
            os.path.join(maps_dir, "scans_0.5m_redownsampled.pcd"),
        ]
    elif args.batch:
        inputs = [os.path.expanduser(p) for p in args.batch]
        outputs = [_auto_output_path(p, args.leaf, args.in_place) for p in inputs]
    elif args.input:
        inp = os.path.expanduser(args.input)
        out = os.path.expanduser(args.output) if args.output else _auto_output_path(
            inp, args.leaf, args.in_place
        )
        inputs, outputs = [inp], [out]
    else:
        parser.print_help()
        return 1

    rc = 0
    for inp, out in zip(inputs, outputs):
        if not os.path.isfile(inp):
            print(f"warning: 跳过（不存在）: {inp}", file=sys.stderr)
            rc = 1
            continue
        if args.in_place and args.backup and os.path.abspath(inp) == os.path.abspath(out):
            bak = inp + ".bak"
            if not os.path.isfile(bak):
                print(f"备份: {bak}")
                shutil.copy2(inp, bak)
        if downsample_one(inp, out, args.leaf, args.estimate_normals) != 0:
            rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
