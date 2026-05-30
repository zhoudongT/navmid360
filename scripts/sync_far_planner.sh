#!/usr/bin/env bash
# 同步 CMU FAR Planner（ROS 2 Humble/Jazzy 分支）到本仓库。
#
# 官方仓库: https://github.com/MichaelFYang/far_planner
# 分支: humble-jazzy
# 论文/说明: https://www.far-planner.com/  https://www.cmu-exploration.com/
#
# 用法:
#   bash scripts/sync_far_planner.sh
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${REPO_ROOT}/far_planner_ws"
URL="https://github.com/MichaelFYang/far_planner.git"
BRANCH="humble-jazzy"

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
echo "  bash ${REPO_ROOT}/scripts/build_far_planner_humble.sh"
echo
echo "启动（替代 EGO）:"
echo "  ros2 launch go2_nav_stack bringup_humble.launch.py local_planner:=far"
