"""可选超时：未设置环境变量时默认无限等待（spin 直到 future 完成）。

用于大地图导出/保存，避免客户端或服务端过早报「保存超时」。
需要上限时在启动前 export 对应变量（秒，正数）；0 / none / inf 表示不限制。

  JIE_NAV_MAP_SAVE_TIMEOUT_SEC   — GUI 等 save_package 响应
  JIE_NAV_MAP_LOAD_TIMEOUT_SEC   — GUI 等 load_package 响应
  JIE_NAV_MAP_EXPORT_TIMEOUT_SEC — map_package_manager 等 planner export_snapshot
  JIE_NAV_MAP_META_TIMEOUT_SEC   — map_package_manager 等 get_navigation_map_meta
"""

from __future__ import annotations

import os


def _timeout_from_env(name: str, default: float | None) -> float | None:
    raw = os.environ.get(name)
    if raw is None:
        return default
    s = str(raw).strip().lower()
    if s in ("", "none", "off", "unlimited", "inf", "infinite"):
        return None
    try:
        v = float(s)
    except ValueError:
        return default
    if v <= 0:
        return None
    return v


def save_navigation_map_client_spin_timeout_sec() -> float | None:
    return _timeout_from_env("JIE_NAV_MAP_SAVE_TIMEOUT_SEC", default=None)


def load_navigation_map_client_spin_timeout_sec() -> float | None:
    return _timeout_from_env("JIE_NAV_MAP_LOAD_TIMEOUT_SEC", default=None)


def export_navigation_snapshot_wait_timeout_sec() -> float | None:
    return _timeout_from_env("JIE_NAV_MAP_EXPORT_TIMEOUT_SEC", default=None)


def get_navigation_map_meta_wait_timeout_sec() -> float | None:
    return _timeout_from_env("JIE_NAV_MAP_META_TIMEOUT_SEC", default=None)
