#!/usr/bin/env bash
# Go2 + Mid-360 + Fast-LIO2-Localization 完整导航一键启动
#
# 用法:
#   ./scripts/start_go2_nav.sh
#   MAP_PACKAGE=/path/to/map PRIOR_PCD=/path/to/map.pcd ./scripts/start_go2_nav.sh
#   ./scripts/start_go2_nav.sh --no-gui --no-rviz
#
# 启动前请确保已编译:
#   bash scripts/build_all_humble.sh
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "错误: 需要 ROS 2 Humble" >&2
  exit 1
fi

# shellcheck source=/dev/null
set +u
source /opt/ros/humble/setup.bash
if [[ -f "${ROOT}/install/setup.bash" ]]; then
  source "${ROOT}/install/setup.bash"
else
  echo "错误: 未找到 install/setup.bash，请先: bash scripts/build_all_humble.sh" >&2
  exit 1
fi
set -u

export CMAKE_PREFIX_PATH="${HOME}/.local/opt/open3d-0.17.0-cxx11:${CMAKE_PREFIX_PATH:-}"

exec "${ROOT}/scripts/go2_nav_oneclick.sh" "$@"
