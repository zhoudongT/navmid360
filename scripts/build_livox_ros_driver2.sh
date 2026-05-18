#!/usr/bin/env bash
# 在「ROS 2 工作区根目录」下仅编译本仓库内的 livox_ros_driver2，且用 --paths 限定扫描范围。
#
# 请勿在 livox_ros_driver2 目录内执行官方 ./build.sh ROS2：该脚本会 cd ../../ 再 colcon，
# 若 livox 位于 dong_3d_nav/livox_ros_driver2，../../ 会落到用户家目录等，导致 colcon 全盘扫包、
# 报大量与 Livox 无关的 python_setup_py / SyntaxError 等错误。
#
# 用法:
#   source /opt/ros/foxy/setup.bash
#   bash scripts/build_livox_ros_driver2.sh /path/to/ros2_ws
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIVOX_DIR="${REPO_ROOT}/livox_ros_driver2"

if [[ ! -f "${LIVOX_DIR}/package.xml" ]]; then
  echo "未找到 ${LIVOX_DIR}，请先: bash scripts/sync_fastlio_mid360_deps.sh" >&2
  exit 1
fi

WS_ROOT="${1:-}"
if [[ -z "${WS_ROOT}" ]] || [[ ! -d "${WS_ROOT}" ]]; then
  echo "用法: bash scripts/build_livox_ros_driver2.sh <ROS2 工作区根目录>" >&2
  echo "示例: bash scripts/build_livox_ros_driver2.sh ~/ros2_ws" >&2
  exit 1
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "请先 source ROS 2 环境，例如: source /opt/ros/foxy/setup.bash" >&2
  exit 1
fi

cd "${WS_ROOT}"
colcon build --symlink-install --packages-select livox_ros_driver2 \
  --paths "${LIVOX_DIR}" \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3

echo
echo "完成。请执行: source ${WS_ROOT}/install/setup.bash"
