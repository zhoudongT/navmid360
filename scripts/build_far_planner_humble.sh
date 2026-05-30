#!/usr/bin/env bash
# 编译 FAR Planner（CMU MichaelFYang/far_planner humble-jazzy）
#
# 前置:
#   bash scripts/sync_far_planner.sh
#   sudo apt install ros-humble-pcl-* libopencv-dev
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAR_ROOT="${REPO_ROOT}/far_planner_ws"
SRC="${FAR_ROOT}/src"

if [[ ! -d "${FAR_ROOT}/.git" ]]; then
  echo "错误: 未找到 ${FAR_ROOT}，请先运行 scripts/sync_far_planner.sh" >&2
  exit 1
fi

# shellcheck source=scripts/dong_ros_env.sh
source "${REPO_ROOT}/scripts/dong_ros_env.sh"
dong_source_humble || exit 1

PKGS=(visibility_graph_msg graph_decoder far_planner)
PATHS=(
  "${SRC}/visibility_graph_msg"
  "${SRC}/graph_decoder"
  "${SRC}/far_planner"
)

echo "[build] colcon build FAR Planner …"
cd "${REPO_ROOT}"
colcon build --symlink-install \
  --packages-select "${PKGS[@]}" \
  --paths "${PATHS[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3

echo
echo "[build] 完成。请 source:"
echo "  source ${REPO_ROOT}/install/setup.bash"
