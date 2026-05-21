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
  "rosbridge_websocket"
  "rosbridge_server"
  "run_rosbridge_safe"
  "no_cache_http_server"
  # 勿杀 start_go2_nav / go2_nav_oneclick：preflight 在 oneclick 内调用时会自杀，栈起不来
)

_kill_patterns TERM "${PATTERNS[@]}"
sleep 2
_kill_patterns KILL "${PATTERNS[@]}"
sleep 1

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -x "${ROOT}/scripts/free_nav_ports.sh" ]]; then
  "${ROOT}/scripts/free_nav_ports.sh" 9090 9091 8888 8080 8081
else
  pkill -KILL -f "rosbridge_websocket" 2>/dev/null || true
  if command -v fuser >/dev/null 2>&1; then
    for _port in 8888 8081 8080 9090 9091; do
      fuser -k "${_port}"/tcp 2>/dev/null || true
    done
    unset _port
  fi
fi
rm -f /tmp/go2_nav_stack.lock /tmp/go2_nav_stack.rosbridge_port 2>/dev/null || true

if pgrep -f "ros2 launch go2_nav_stack" >/dev/null 2>&1; then
  echo "警告: 仍有残留 launch 进程，可手动: ps aux | grep go2_nav"
  exit 1
fi

echo ">>> 导航栈已结束，可重新运行 ./scripts/go2_nav_oneclick.sh"
