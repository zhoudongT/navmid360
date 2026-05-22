#!/usr/bin/env bash
# 编译 ego-planner-swarm（Go2 实机仅需 planner + quadrotor_msgs，不含仿真器）
#
# 前置:
#   bash scripts/sync_ego_planner_swarm.sh
#   sudo apt install libarmadillo-dev ros-humble-pcl-* ros-humble-cv-bridge
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EGO_ROOT="${REPO_ROOT}/ego-planner-swarm"
SRC_PLANNER="${EGO_ROOT}/src/planner"
SRC_MSGS="${EGO_ROOT}/src/uav_simulator/Utils/quadrotor_msgs"

if [[ ! -d "${EGO_ROOT}/.git" ]]; then
  echo "错误: 未找到 ${EGO_ROOT}，请先运行 scripts/sync_ego_planner_swarm.sh" >&2
  exit 1
fi

# shellcheck source=scripts/dong_ros_env.sh
source "${REPO_ROOT}/scripts/dong_ros_env.sh"
dong_source_humble || exit 1

if ! dpkg -s libarmadillo-dev >/dev/null 2>&1; then
  echo "提示: 需要 libarmadillo-dev，请执行: sudo apt install libarmadillo-dev" >&2
fi

export CMAKE_PREFIX_PATH="${AMENT_PREFIX_PATH:-}"

PKGS=(
  quadrotor_msgs
  plan_env
  path_searching
  bspline_opt
  traj_utils
  ego_planner
)

PATHS=(
  "${SRC_MSGS}"
  "${SRC_PLANNER}/plan_env"
  "${SRC_PLANNER}/path_searching"
  "${SRC_PLANNER}/bspline_opt"
  "${SRC_PLANNER}/traj_utils"
  "${SRC_PLANNER}/plan_manage"
)

echo "[build] colcon build ego_planner 栈 …"
cd "${REPO_ROOT}"
colcon build --symlink-install \
  --packages-select "${PKGS[@]}" \
  --paths "${PATHS[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3

echo
echo "[build] 完成。请 source:"
echo "  source ${REPO_ROOT}/install/setup.bash"
H