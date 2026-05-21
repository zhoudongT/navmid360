#!/usr/bin/env bash
# 仅启动定位层：Livox + Fast-LIO2-Localization + ICP + TF + Nav2 MPPI（无网页/Qt 地图管理）
#
# 用法:
#   ./scripts/start_fastlio_bringup.sh
#   PRIOR_PCD=/home/allen/maps/scans_0.5m.pcd ./scripts/start_fastlio_bringup.sh
#   ./scripts/start_fastlio_bringup.sh --no-rviz
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

PRIOR_PCD="${PRIOR_PCD:-/home/allen/maps/scans_0.5m.pcd}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-true}"
RELOC_AUTO_START="${RELOC_AUTO_START:-false}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-rviz) LAUNCH_RVIZ=false; shift ;;
    -h|--help)
      echo "用法: $0 [--no-rviz]"
      echo "环境变量: PRIOR_PCD LAUNCH_RVIZ RELOC_AUTO_START"
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

echo ">>> 启动 bringup（FAST-LIO2 + ICP + TF + MPPI）"
echo "    先验 PCD: ${PRIOR_PCD}"

exec ros2 launch go2_nav_stack bringup_humble.launch.py \
  prior_pcd_path:="${PRIOR_PCD}" \
  launch_rviz:="${LAUNCH_RVIZ}" \
  reloc_auto_start:="${RELOC_AUTO_START}"
