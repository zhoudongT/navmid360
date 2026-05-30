#!/usr/bin/env bash
# Go2 一键启动（走廊 + Nav2 MPPI 局部避障）
#
# 等价于: LOCAL_PLANNER=mppi ./scripts/go2_nav_oneclick.sh "$@"
#
# 依赖:
#   sudo apt install -y ros-humble-nav2-mppi-controller ros-humble-nav2-controller \
#                       ros-humble-nav2-costmap-2d
#
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export LOCAL_PLANNER=mppi
if command -v ros2 &>/dev/null && [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  [[ -f "$ROOT/install/setup.bash" ]] && source "$ROOT/install/setup.bash"
  RVIZ_MPPI="$(ros2 pkg prefix go2_nav_stack 2>/dev/null)/share/go2_nav_stack/config/rviz_go2_nav_mppi.rviz"
fi
export RVIZ_CONFIG="${RVIZ_CONFIG:-${RVIZ_MPPI:-$ROOT/go2_nav_stack/config/rviz_go2_nav_mppi.rviz}}"
exec "$ROOT/scripts/go2_nav_oneclick.sh" "$@"
