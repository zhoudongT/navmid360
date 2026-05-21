#!/usr/bin/env bash
# 强制结束 Go2 导航一键启动相关进程（重启前请先执行）
set -u

echo ">>> 结束 Go2 导航栈进程..."

# 先 SIGTERM，再 SIGKILL
_kill_patterns() {
  local sig="$1"
  shift
  for pat in "$@"; do
    if pgrep -f "$pat" >/dev/null 2>&1; then
      echo "  ${sig} ${pat}"
      pkill -"$sig" -f "$pat" 2>/dev/null || true
    fi
  done
}

PATTERNS=(
  "ros2 launch go2_nav_stack"
  "ros2 launch dong_octomap"
  "ros2 launch octo_planner"
  "ros2 launch icp_relocalization"
  "controller_server"
  "nav2_costmap_2d"
  "nav2_controller_bringup"
  "mppi_path_follower"
  "map_viewer_gui"
  "map_save_gui"
  "dong_path_node"
  "icp_node"
  "kiss_icp"
  "livox_ros_driver2"
)

_kill_patterns TERM "${PATTERNS[@]}"
sleep 2
_kill_patterns KILL "${PATTERNS[@]}"
sleep 1

# 释放可能被占用的端口（rosbridge 网页）
if command -v fuser >/dev/null 2>&1; then
  fuser -k 8080/tcp 2>/dev/null || true
fi

if pgrep -f "ros2 launch go2_nav_stack" >/dev/null 2>&1; then
  echo "警告: 仍有残留 launch 进程，可手动: ps aux | grep go2_nav"
  exit 1
fi

echo ">>> 导航栈已结束，可重新运行 ./scripts/go2_nav_oneclick.sh"
