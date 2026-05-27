#!/usr/bin/env bash
# 安全 source 本工作区（兼容 bash set -u / nounset）
#
# 用法:
#   source ~/navmid360/scripts/source_navmid360.sh
#   # 或
#   . scripts/source_navmid360.sh
#
# 勿在 set -u 下直接: source install/setup.bash  （会报 COLCON_TRACE 未绑定）

_NAVMID360_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/dong_ros_env.sh
source "${_NAVMID360_ROOT}/scripts/dong_ros_env.sh"
dong_source_workspace "${_NAVMID360_ROOT}"
unset _NAVMID360_ROOT
