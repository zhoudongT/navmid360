#!/usr/bin/env bash
# 在 ROS 2 工作区根目录下仅编译 livox_ros_driver2（--paths 限定范围，避免全盘 colcon）。
#
# 请勿在 livox 目录内对整仓执行官方 ./build.sh ROS2 且未 source humble：
# CMake 需要 -DDISTRO_ROS=humble，否则 LIVOX_INTERFACES_INCLUDE_DIRECTORIES 为 NOTFOUND。
#
# 用法:
#   source /opt/ros/humble/setup.bash
#   bash scripts/build_livox_ros_driver2.sh              # 工作区默认为本仓库根目录
#   bash scripts/build_livox_ros_driver2.sh ~/ros2_ws    # 指定工作区
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIVOX_DIR=""
for _candidate in "${REPO_ROOT}/src/livox_ros_driver2" "${REPO_ROOT}/livox_ros_driver2"; do
  if [[ -f "${_candidate}/package.xml" ]]; then
    LIVOX_DIR="${_candidate}"
    break
  fi
done
unset _candidate

if [[ -z "${LIVOX_DIR}" ]]; then
  echo "未找到 livox_ros_driver2，请先: bash scripts/sync_fastlio_mid360_deps.sh" >&2
  exit 1
fi

WS_ROOT="${1:-${REPO_ROOT}}"
if [[ ! -d "${WS_ROOT}" ]]; then
  echo "工作区目录不存在: ${WS_ROOT}" >&2
  exit 1
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "请先 source ROS 2 环境，例如: source /opt/ros/humble/setup.bash" >&2
  exit 1
fi

cd "${WS_ROOT}"
colcon build --symlink-install --packages-select livox_ros_driver2 \
  --paths "${LIVOX_DIR}" \
  --cmake-args \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS="${ROS_DISTRO}"

echo
echo "完成。请执行: source ${WS_ROOT}/install/setup.bash"
