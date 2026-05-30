#!/usr/bin/env bash
# Go2 一键启动（CMU FAR Planner 版）
#
# 等价于: LOCAL_PLANNER=far ./scripts/go2_nav_oneclick.sh "$@"
#
# 首次使用前编译 FAR:
#   bash scripts/sync_far_planner.sh
#   bash scripts/build_far_planner_humble.sh
#
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export LOCAL_PLANNER=far
# 安装后优先用 share 路径；开发时回退到源码 config
if command -v ros2 &>/dev/null && [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  [[ -f "$ROOT/install/setup.bash" ]] && source "$ROOT/install/setup.bash"
  RVIZ_FAR="$(ros2 pkg prefix go2_nav_stack 2>/dev/null)/share/go2_nav_stack/config/rviz_go2_nav_far.rviz"
fi
export RVIZ_CONFIG="${RVIZ_CONFIG:-${RVIZ_FAR:-$ROOT/go2_nav_stack/config/rviz_go2_nav_far.rviz}}"
exec "$ROOT/scripts/go2_nav_oneclick.sh" "$@"
