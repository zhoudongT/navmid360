#!/usr/bin/env bash
# 一键：检查依赖 -> 编译仿真包 -> 启动 Gazebo + FAST-LIO 建图
#
# 用法:
#   bash scripts/run_mid360_sim_slam.sh
#   bash scripts/run_mid360_sim_slam.sh --no-build   # 跳过编译
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DO_BUILD=true

for arg in "$@"; do
  case "${arg}" in
    --no-build) DO_BUILD=false ;;
    -h|--help)
      echo "用法: bash scripts/run_mid360_sim_slam.sh [--no-build]"
      exit 0
      ;;
    *) echo "未知参数: ${arg}" >&2; exit 1 ;;
  esac
done

if [[ -z "${ROS_DISTRO:-}" ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi

bash "${REPO_ROOT}/scripts/sync_mid360_gazebo_sim.sh"

if [[ "${DO_BUILD}" == "true" ]]; then
  bash "${REPO_ROOT}/scripts/build_gazebo_sim.sh"
fi

# shellcheck source=/dev/null
source "${REPO_ROOT}/install/setup.bash"

if ! ros2 pkg prefix fast_lio &>/dev/null; then
  echo "WARN: 未找到 fast_lio，SLAM 将失败。请先编译 Fast-LIO2-Localization/FAST_LIO。" >&2
fi

exec ros2 launch dong_gazebo_sim mid360_sim_slam.launch.py
