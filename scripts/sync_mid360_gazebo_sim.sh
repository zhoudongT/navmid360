#!/usr/bin/env bash
# 克隆 Mid-360 Gazebo 仿真插件（ROS 2 Humble）到 src/ros2_livox_simulation
#
# 用法:
#   bash scripts/sync_mid360_gazebo_sim.sh
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${REPO_ROOT}/src/ros2_livox_simulation"
URL="https://github.com/LihanChen2004/livox_laser_simulation_ros2.git"
BRANCH="main"

_git() {
  env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY \
    -u all_proxy -u ALL_PROXY git "$@"
}

if [[ -d "${DEST}/.git" ]]; then
  echo "[sync-sim] 已存在 ${DEST}，尝试 git pull…"
  _git -C "${DEST}" pull --ff-only || echo "[sync-sim] pull 失败可忽略，继续使用本地副本。"
else
  if [[ -e "${DEST}" ]]; then
    echo "[sync-sim] 错误: ${DEST} 存在且不是 git 仓库。" >&2
    exit 1
  fi
  mkdir -p "$(dirname "${DEST}")"
  echo "[sync-sim] git clone ${URL} -> ${DEST}"
  _git clone --depth 1 -b "${BRANCH}" "${URL}" "${DEST}"
fi

echo
echo "[sync-sim] 完成。下一步:"
echo "  source /opt/ros/humble/setup.bash"
echo "  bash scripts/build_gazebo_sim.sh"
