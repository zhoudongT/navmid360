#!/usr/bin/env bash
# 在已克隆的 FAST_LIO_ROS2 目录上应用本仓库 CMake 补丁（ROS 2 发行版解析 + rosidl 兼容）。
# 在 colcon build fast_lio 之前执行一次即可；可重复执行。
#
# 用法:
#   bash scripts/apply_fast_lio_ros2_patches.sh
#   bash scripts/apply_fast_lio_ros2_patches.sh /path/to/fast_lio
#
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET="${1:-}"
if [[ -z "${TARGET}" ]]; then
  for d in "${REPO_ROOT}/fast_lio" "${REPO_ROOT}/src/fast_lio"; do
    if [[ -f "${d}/CMakeLists.txt" ]]; then
      TARGET="${d}"
      break
    fi
  done
fi
if [[ -z "${TARGET}" ]] || [[ ! -f "${TARGET}/CMakeLists.txt" ]]; then
  echo "未找到 fast_lio/CMakeLists.txt。请先克隆（scripts/sync_fastlio_mid360_deps.sh）或传入路径。" >&2
  exit 1
fi
exec python3 "${REPO_ROOT}/scripts/patch_fast_lio_ros2_cmake.py" "${TARGET}"
