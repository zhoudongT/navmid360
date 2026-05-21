#!/usr/bin/env bash
# 诊断：lio_odom -> state_estimation -> TF -> costmap
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
echo "=== ② 平面滤波输出（costmap 跟随的前提）==="
timeout 5 ros2 topic hz /state_estimation 2>&1 | tail -3 || echo "  无 /state_estimation — 检查 planar_odom_filter_node 日志"

echo ""
echo "=== ③ TF map->base_link（走动时 translation 应变化）==="
timeout 2 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -E "Translation|At time" | head -6

echo ""
echo "=== ④ costmap（须指定消息类型）==="
timeout 4 ros2 topic hz /costmap/costmap 2>&1 | tail -2 || echo "  无数据"
echo "  查看一帧: ros2 topic echo /costmap/costmap --field header.frame_id --once"

echo ""
echo "=== ⑤ 采样 position（正确语法用 --field）==="
timeout 3 ros2 topic echo /state_estimation --field pose.pose.position --once 2>/dev/null || echo "  state_estimation 无数据"
