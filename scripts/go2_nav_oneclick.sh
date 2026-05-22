#!/usr/bin/env bash
# Go2 实车一键启动：Livox + FAST-LIO2 + ICP + TF + 网页规划 + Qt 地图管理 + Nav2 MPPI
#
# 重定位（默认）：不自动 Scan Context，请在 Qt map_viewer 中：
#   1. 点「当前姿态」-> 在地图上点机器人位置
#   2. 再点朝向 -> 发布 /initialpose，等待 ICP 收敛（/relocalization/status=RELOCALIZED）
#
# 用法:
#   ./scripts/go2_nav_oneclick.sh
#   MAP_PACKAGE=/path/to/octomap_pkg PRIOR_PCD=/path/to/map.pcd ./scripts/go2_nav_oneclick.sh
#   ./scripts/go2_nav_oneclick.sh --no-gui --no-rviz
#   RELOC_AUTO_START=true ./scripts/go2_nav_oneclick.sh   # 改回开机自动 Scan Context

set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

MAP_PACKAGE="${MAP_PACKAGE:-/home/allen/maps/window_filtered_all}"
PRIOR_PCD="${PRIOR_PCD:-/home/allen/maps/scans_0.5m.pcd}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-true}"
LAUNCH_MAP_GUI="${LAUNCH_MAP_GUI:-true}"
LAUNCH_ROSBRIDGE="${LAUNCH_ROSBRIDGE:-true}"
LAUNCH_WEB="${LAUNCH_WEB:-true}"
WEB_PORT="${WEB_PORT:-8888}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
RELOC_AUTO_START="${RELOC_AUTO_START:-false}"
FAST_START="${FAST_START:-1}"
ROSBRIDGE_DELAY_SEC="${ROSBRIDGE_DELAY_SEC:-2.0}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-gui) LAUNCH_MAP_GUI=false; shift ;;
    --no-rviz) LAUNCH_RVIZ=false; shift ;;
    --no-web-bridge) LAUNCH_ROSBRIDGE=false; shift ;;
    --no-web) LAUNCH_WEB=false; shift ;;
    -h|--help)
      echo "用法: $0 [--no-gui] [--no-rviz] [--no-web] [--no-web-bridge]"
      echo "环境变量: MAP_PACKAGE PRIOR_PCD FAST_START ROSBRIDGE_DELAY_SEC RELOC_AUTO_START ..."
      exit 0
      ;;
    *) echo "未知参数: $1"; exit 1 ;;
  esac
done

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "错误: 未找到 ROS2 Humble: /opt/ros/humble/setup.bash"
  exit 1
fi
# shellcheck source=/dev/null
set +u
source /opt/ros/humble/setup.bash

if [[ ! -f "$ROOT/install/livox_ros_driver2/share/livox_ros_driver2/package.xml" ]]; then
  echo "错误: livox_ros_driver2 未编译。须先编 Livox，再编其余包:"
  echo "  cd $ROOT"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  bash scripts/build_all_humble.sh --livox-only"
  echo "  source install/setup.bash"
  echo "  bash scripts/build_all_humble.sh --rest-only"
  exit 1
fi
if [[ ! -f "$ROOT/install/fast_lio/share/fast_lio/package.xml" ]]; then
  echo "错误: fast_lio 未编译，请运行:"
  echo "  cd $ROOT && bash scripts/build_all_humble.sh"
  exit 1
fi
if [[ ! -f "$ROOT/install/setup.bash" ]]; then
  echo "错误: 工作区未完整编译，请运行:"
  echo "  cd $ROOT && bash scripts/build_all_humble.sh"
  exit 1
fi
# shellcheck source=/dev/null
source "$ROOT/install/setup.bash"

if [[ ! -d "$MAP_PACKAGE" ]]; then
  echo "错误: OctoMap 地图包不存在: $MAP_PACKAGE"
  exit 1
fi
if [[ ! -f "$MAP_PACKAGE/meta.yaml" ]]; then
  echo "错误: 地图包缺少 meta.yaml: $MAP_PACKAGE"
  exit 1
fi
if [[ ! -f "$PRIOR_PCD" ]]; then
  echo "错误: 先验 PCD 不存在: $PRIOR_PCD"
  exit 1
fi

if [[ "$LAUNCH_ROSBRIDGE" == "true" ]] && ! ros2 pkg prefix rosbridge_server &>/dev/null; then
  echo "警告: 未安装 rosbridge，网页无法连 ROS。请执行:"
  echo "  sudo apt install -y ros-humble-rosbridge-server"
  exit 1
fi

PREFLIGHT="$ROOT/scripts/go2_nav_preflight.sh"
if [[ -x "$PREFLIGHT" ]]; then
  FAST_START="$FAST_START" ROSBRIDGE_PORT="$ROSBRIDGE_PORT" WEB_PORT="$WEB_PORT" "$PREFLIGHT"
  if [[ -f /tmp/go2_nav_stack.rosbridge_port ]]; then
    ROSBRIDGE_PORT="$(cat /tmp/go2_nav_stack.rosbridge_port)"
  fi
else
  KILL_SCRIPT="$ROOT/scripts/go2_nav_kill.sh"
  if [[ -x "$KILL_SCRIPT" ]]; then
    echo ">>> 先结束旧进程..."
    "$KILL_SCRIPT" || true
    sleep 2
  fi
fi

LOCK_FILE="/tmp/go2_nav_stack.lock"
echo $$ >"${LOCK_FILE}"
trap 'rm -f "${LOCK_FILE}"' EXIT

echo "=========================================="
echo " Go2 导航一键启动"
echo " 地图包:     $MAP_PACKAGE"
echo " 先验 PCD:   $PRIOR_PCD"
echo " 网页:       $([ "$LAUNCH_WEB" = true ] && echo "http://127.0.0.1:${WEB_PORT}" || echo '未启动 (--no-web)')"
echo " rosbridge:  ws://127.0.0.1:${ROSBRIDGE_PORT}"
echo " Qt Manager: $LAUNCH_MAP_GUI"
echo " RViz:       $LAUNCH_RVIZ"
echo " 重定位:     $([ "$RELOC_AUTO_START" = true ] && echo '开机自动 ScanContext' || echo '手动 Qt「当前姿态」点击')"
echo " 里程计:     FAST-LIO2 + ICP 先验图"
echo " 局部规划:   EGO Planner；全局 /planned_path -> /move_base_simple/goal"
echo " 快速启动:   FAST_START=${FAST_START}  rosbridge_delay=${ROSBRIDGE_DELAY_SEC}s"
echo "=========================================="

exec ros2 launch go2_nav_stack go2_full_stack.launch.py \
  map_package_dir:="$MAP_PACKAGE" \
  prior_pcd_path:="$PRIOR_PCD" \
  launch_rviz:="$LAUNCH_RVIZ" \
  launch_map_gui:="$LAUNCH_MAP_GUI" \
  launch_web:="$LAUNCH_WEB" \
  launch_rosbridge:="$LAUNCH_ROSBRIDGE" \
  web_http_port:="$WEB_PORT" \
  rosbridge_port:="$ROSBRIDGE_PORT" \
  reloc_auto_start:="$RELOC_AUTO_START" \
  rosbridge_delay_sec:="$ROSBRIDGE_DELAY_SEC"
