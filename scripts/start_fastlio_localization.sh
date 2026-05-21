#!/usr/bin/env bash
# 最小链路：Mid-360 + Fast-LIO2-Localization（Livox + ICP + FAST-LIO），不含 go2_nav_stack
#
# 用法:
#   ./scripts/start_fastlio_localization.sh
#   PRIOR_PCD=/path/to/map.pcd ./scripts/start_fastlio_localization.sh
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

PRIOR_PCD="${PRIOR_PCD:-/home/allen/maps/scans_0.5m.pcd}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-true}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-rviz) LAUNCH_RVIZ=false; shift ;;
    -h|--help)
      echo "用法: $0 [--no-rviz]"
      echo "环境变量: PRIOR_PCD LAUNCH_RVIZ"
      exit 0
      ;;
    *) echo "未知参数: $1" >&2; exit 1 ;;
  esac
done

# shellcheck source=/dev/null
set +u
source /opt/ros/humble/setup.bash
source "${ROOT}/install/setup.bash"
set -u

if [[ ! -f "${PRIOR_PCD}" ]]; then
  echo "错误: 先验 PCD 不存在: ${PRIOR_PCD}" >&2
  exit 1
fi

"${ROOT}/scripts/go2_nav_kill.sh" 2>/dev/null || true
sleep 1

echo ">>> Fast-LIO2-Localization（dong_octomap）"
echo "    先验 PCD: ${PRIOR_PCD}"

exec ros2 launch dong_octomap mid360_fastlio2_localization.launch.py \
  prior_pcd_path:="${PRIOR_PCD}" \
  locate_in_prior_map:=true \
  launch_rviz:="${LAUNCH_RVIZ}"
