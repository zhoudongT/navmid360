#!/usr/bin/env bash
# Gazebo Go2 四足真狗 + 真实 2_storey 地图 + FAST-LIO2 + 导航全栈
#
# 地图目录: /home/allen/maps/2_storey（OctoMap 包 + 先验 PCD）
#
# 用法:
#   ./scripts/run_legged_sim_2_storey.sh              # 全栈（Gazebo + 定位 + 地图GUI + 网页）
#   ./scripts/run_legged_sim_2_storey.sh --sim-only   # 仅仿真 + RViz 点云
#   ./scripts/run_legged_sim_2_storey.sh --no-gui   # 不启 Gazebo GUI
#
# 环境变量:
#   MAPS_ROOT=/home/allen/maps
#   MAP_PACKAGE=/home/allen/maps/2_storey
#   PRIOR_PCD=/home/allen/maps/2_storey/global_map_gazebo.pcd

set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

MAPS_ROOT="${MAPS_ROOT:-/home/allen/maps}"
MAP_PACKAGE="${MAP_PACKAGE:-${MAPS_ROOT}/2_storey}"
PRIOR_PCD="${PRIOR_PCD:-${MAP_PACKAGE}/global_map_gazebo.pcd}"
LAUNCH_GAZEBO_GUI="${LAUNCH_GAZEBO_GUI:-true}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-true}"
LAUNCH_MAP_GUI="${LAUNCH_MAP_GUI:-true}"
LAUNCH_WEB="${LAUNCH_WEB:-true}"
LAUNCH_ROSBRIDGE="${LAUNCH_ROSBRIDGE:-true}"
RELOC_AUTO_START="${RELOC_AUTO_START:-true}"
WEB_PORT="${WEB_PORT:-8081}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9091}"
SIM_ONLY=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sim-only) SIM_ONLY=true; LAUNCH_MAP_GUI=false; LAUNCH_WEB=false; LAUNCH_ROSBRIDGE=false; shift ;;
    --no-gui) LAUNCH_GAZEBO_GUI=false; shift ;;
    --no-rviz) LAUNCH_RVIZ=false; shift ;;
    -h|--help)
      cat <<'EOF'
用法: ./scripts/run_legged_sim_2_storey.sh [--sim-only] [--no-gui] [--no-rviz]

  --sim-only   仅 Gazebo 真狗 + FAST-LIO2 + RViz（不启导航/地图GUI）
  --no-gui     不显示 Gazebo 窗口
  --no-rviz    不启动 RViz

默认地图:
  MAP_PACKAGE=/home/allen/maps/2_storey
  PRIOR_PCD=.../global_map_gazebo.pcd
EOF
      exit 0
      ;;
    *) echo "未知参数: $1"; exit 1 ;;
  esac
done

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "错误: 未找到 ROS2 Humble"
  exit 1
fi

# shellcheck source=/dev/null
source "${ROOT}/scripts/source_navmid360.sh"

if [[ ! -d "$MAP_PACKAGE" ]] || [[ ! -f "$MAP_PACKAGE/meta.yaml" ]]; then
  echo "错误: 地图包无效: $MAP_PACKAGE（需含 meta.yaml）"
  exit 1
fi
if [[ ! -f "$PRIOR_PCD" ]]; then
  PRIOR_PCD="${MAP_PACKAGE}/global_map.pcd"
  if [[ ! -f "$PRIOR_PCD" ]]; then
    echo "错误: 找不到先验 PCD: ${MAP_PACKAGE}/global_map_gazebo.pcd 或 global_map.pcd"
    exit 1
  fi
  echo ">>> 使用 fallback 先验: $PRIOR_PCD"
fi

if ! ros2 pkg prefix champ_bringup &>/dev/null; then
  echo ">>> 未检测到 CHAMP，正在编译..."
  bash "${ROOT}/scripts/build_go2_champ_gazebo.sh"
  # shellcheck source=/dev/null
  source "${ROOT}/scripts/source_navmid360.sh"
fi

echo ">>> 清理旧 Gazebo / 导航进程..."
bash "${ROOT}/scripts/go2_nav_kill.sh" 2>/dev/null || true
killall -q gzserver gzclient 2>/dev/null || true
sleep 2

echo "=========================================="
echo " Go2 四足 Gazebo + 真实地图"
echo " 地图包:   $MAP_PACKAGE"
echo " 先验PCD:  $PRIOR_PCD"
echo " 模式:     $([ "$SIM_ONLY" = true ] && echo '仅仿真+RViz' || echo '仿真+导航全栈')"
echo " RViz:     $LAUNCH_RVIZ"
echo " 地图GUI:  $LAUNCH_MAP_GUI"
echo " 网页:     $([ "$LAUNCH_WEB" = true ] && echo "http://127.0.0.1:${WEB_PORT}" || echo '关')"
echo "=========================================="

if [[ "$SIM_ONLY" == true ]]; then
  exec ros2 launch dong_gazebo_sim mid360_sim_legged_fastlio2.launch.py \
    gui:="$LAUNCH_GAZEBO_GUI" \
    launch_rviz:="$LAUNCH_RVIZ" \
    prior_pcd_path:="$PRIOR_PCD" \
    initial_x:=1.0 initial_y:=-2.5 initial_z:=0.275
fi

exec ros2 launch dong_gazebo_sim mid360_sim_legged_go2_nav.launch.py \
  prior_pcd_path:="$PRIOR_PCD" \
  map_package_dir:="$MAP_PACKAGE" \
  maps_root:="$MAPS_ROOT" \
  gui:="$LAUNCH_GAZEBO_GUI" \
  launch_rviz:="$LAUNCH_RVIZ" \
  launch_map_gui:="$LAUNCH_MAP_GUI" \
  launch_web:="$LAUNCH_WEB" \
  launch_rosbridge:="$LAUNCH_ROSBRIDGE" \
  web_http_port:="$WEB_PORT" \
  rosbridge_port:="$ROSBRIDGE_PORT" \
  reloc_auto_start:="$RELOC_AUTO_START" \
  nav_stack_delay_sec:=8.0 \
  initial_x:=1.0 initial_y:=-2.5 initial_z:=0.275
