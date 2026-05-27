#!/usr/bin/env bash
# 诊断：lio_odom -> state_estimation -> TF -> EGO 局部规划
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
[[ -f "${ROOT}/install/setup.bash" ]] && source "${ROOT}/install/setup.bash"

ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start
sleep 1

echo "=== ① FAST-LIO 原始里程计 ==="
timeout 5 ros2 topic hz /lio_odom_raw 2>&1 | tail -3 || echo "  无 /lio_odom_raw"

echo ""
echo "=== ② tf_manager 里程计（EGO 输入经 bridge 转发）==="
timeout 5 ros2 topic hz /state_estimation 2>&1 | tail -3 || echo "  无 /state_estimation"

echo ""
echo "=== ③ TF map->base_link（走动时 translation 应变化）==="
timeout 2 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -E "Translation|At time" | head -6

echo ""
echo "=== ④ EGO 局部规划（非 Nav2 costmap）==="
if ! ros2 interface show traj_utils/msg/Bspline &>/dev/null; then
  echo "  错误: 未加载 traj_utils，请先: source ${ROOT}/install/setup.bash"
else
  echo "  traj_utils/msg/Bspline OK"
fi
timeout 4 ros2 topic hz /drone_0_planning/bspline 2>&1 | tail -2 || echo "  无 bspline（EGO 未进入 EXEC_TRAJ 或规划失败）"
timeout 4 ros2 topic hz /drone_0_planning/pos_cmd 2>&1 | tail -2 || echo "  无 pos_cmd"
timeout 4 ros2 topic hz /ego_local_path 2>&1 | tail -2 || echo "  无 /ego_local_path"
timeout 4 ros2 topic hz /cmd_vel 2>&1 | tail -2 || echo "  无 /cmd_vel"

echo ""
echo "=== ⑤ 全局路径 ===="
timeout 3 ros2 topic echo /planned_path --field header.frame_id --once 2>/dev/null || echo "  无 /planned_path"
