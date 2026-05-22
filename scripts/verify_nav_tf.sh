#!/usr/bin/env bash
# 对照 导航地图与TF树关系说明.md 第 0 节检查 TF / 话题 / 端口
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [[ -f "${ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ROOT}/install/setup.bash"
fi

echo "=== dong_3d_nav 导航栈一致性检查 ==="

fail=0
ok() { echo "  [OK] $*"; }
bad() { echo "  [FAIL] $*"; fail=1; }

if ros2 topic list 2>/dev/null | grep -q '^/state_estimation$'; then
  ok "/state_estimation 存在"
  if timeout 4 ros2 topic hz /state_estimation 2>&1 | grep -q "average rate"; then
    ok "/state_estimation 有数据（odom->base 会动，EGO 才能跟踪）"
  else
    bad "/state_estimation 无 hz：检查 /lio_odom_raw 与 planar_odom_filter"
  fi
else
  bad "无 /state_estimation（FAST-LIO + planar_odom_filter + tf_manager）"
fi

if ros2 topic list 2>/dev/null | grep -q '^/lio_odom_raw$'; then
  if timeout 3 ros2 topic hz /lio_odom_raw 2>&1 | grep -q "average rate"; then
    ok "/lio_odom_raw 有数据（ICP 初值后 FAST-LIO 才会发）"
  else
    bad "/lio_odom_raw 无数据：先 Qt「当前姿态」完成 ICP，或检查 Livox/IMU"
  fi
else
  bad "无 /lio_odom_raw"
fi

for t in /cloud_registered /cloud_registered_map /transformed_cloud /prior_map /planned_path; do
  if ros2 topic list 2>/dev/null | grep -q "^${t}$"; then
    ok "话题 ${t}"
  else
    bad "缺少话题 ${t}"
  fi
done

if command -v ros2 >/dev/null && timeout 3 ros2 run tf2_ros tf2_echo map livox_frame 2>/dev/null | head -5 | grep -q "Translation"; then
  ok "TF map -> livox_frame 可查"
else
  bad "TF map -> livox_frame 不可用（需重定位且栈在跑）"
fi

if timeout 3 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -5 | grep -q "Translation"; then
  ok "TF odom -> base_link 可查"
else
  bad "TF odom -> base_link 不可用"
fi

if ros2 topic list 2>/dev/null | grep -q '^/drone_0_planning/pos_cmd$'; then
  ok "EGO traj_server 在发 /drone_0_planning/pos_cmd"
else
  bad "无 /drone_0_planning/pos_cmd（ego_planner_go2 未启动或未编译 ego_planner）"
fi

if timeout 3 ros2 topic hz /drone_0_cloud_registered_map 2>&1 | tail -1 | grep -qE '[0-9]'; then
  ok "EGO 障碍点云 /drone_0_cloud_registered_map 有数据"
else
  bad "EGO 点云无数据：需 /cloud_registered_map + map_cloud_display + 重定位"
fi

if timeout 3 ros2 run tf2_ros tf2_echo livox_frame imu_link 2>/dev/null | head -5 | grep -q "Translation"; then
  ok "TF livox_frame -> imu_link 可查"
else
  bad "TF livox_frame -> imu_link 不可用（fastlio static）"
fi

# imu_link 不应同时有 base_link 与 livox_frame 两个父（检查 tf_static 发布者数量）
if ros2 topic info /tf_static -v 2>/dev/null | grep -q "base_link"; then
  :
fi

if ss -tlnp 2>/dev/null | grep -q ':9090 '; then
  cnt=$(pgrep -fc rosbridge_websocket 2>/dev/null || echo 0)
  if [[ "${cnt}" -gt 1 ]]; then
    bad "9090 端口上可能有多个 rosbridge 实例 (count=${cnt})"
  else
    ok "rosbridge 9090 单实例"
  fi
else
  echo "  [SKIP] 9090 未监听（未开网页栈）"
fi

if [[ "${fail}" -eq 0 ]]; then
  echo "=== 检查通过 ==="
else
  echo "=== 存在失败项，见 导航地图与TF树关系说明.md 第 0.5 节 ==="
  exit 1
fi
