#!/usr/bin/env bash
# 同步 ZJU FAST-Lab ego-planner-swarm（ROS2 分支）到本仓库根目录。
#
# 官方仓库: https://github.com/ZJU-FAST-Lab/ego-planner-swarm
# 使用分支: ros2_version（README 中 ROS2 说明）
#
# 用法:
#   bash scripts/sync_ego_planner_swarm.sh
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${REPO_ROOT}/ego-planner-swarm"
URL="https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git"
BRANCH="ros2_version"

clone_shallow() {
  if [[ -d "${DEST}/.git" ]]; then
    echo "[sync] 已存在 ${DEST}"
    echo "[sync] 更新: git -C ${DEST} fetch && git checkout ${BRANCH} && git pull"
    git -C "${DEST}" fetch --depth 1 origin "${BRANCH}" 2>/dev/null || true
    git -C "${DEST}" checkout "${BRANCH}" 2>/dev/null || true
    git -C "${DEST}" pull --ff-only 2>/dev/null || echo "[sync] pull 跳过（无网络或已最新）"
    return 0
  fi
  if [[ -e "${DEST}" ]]; then
    echo "[sync] 错误: ${DEST} 存在且非 git 仓库" >&2
    exit 1
  fi
  echo "[sync] git clone -b ${BRANCH} ${URL} -> ${DEST}"
  git clone --depth 1 -b "${BRANCH}" "${URL}" "${DEST}"
}

clone_shallow

echo
echo "[sync] 完成。编译:"
echo "  bash ${REPO_ROOT}/scripts/build_ego_planner_swarm_humble.sh"
echo
echo "建议 DDS（官方 README）:"
echo "  sudo apt install ros-humble-rmw-cyclonedds-cpp"
echo "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
