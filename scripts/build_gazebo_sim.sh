#!/usr/bin/env bash
# 编译 Mid-360 Gazebo 仿真相关包：ros2_livox_simulation + dong_gazebo_sim
#
# 前置:
#   - source /opt/ros/humble/setup.bash
#   - bash scripts/sync_mid360_gazebo_sim.sh
#   - livox_ros_driver2、fast_lio 已编译（仿真发布 CustomMsg 类型依赖 livox_ros_driver2）
#
# 用法:
#   bash scripts/build_gazebo_sim.sh
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "请先: source /opt/ros/humble/setup.bash" >&2
  exit 1
fi

LIVOX_SIM="${REPO_ROOT}/src/ros2_livox_simulation"
if [[ ! -f "${LIVOX_SIM}/package.xml" ]]; then
  echo "未找到 ${LIVOX_SIM}，请先: bash scripts/sync_mid360_gazebo_sim.sh" >&2
  exit 1
fi

PATHS=("${LIVOX_SIM}" "${REPO_ROOT}/dong_gazebo_sim")

# livox_ros_driver2（消息类型）
for _d in "${REPO_ROOT}/src/livox_ros_driver2" "${REPO_ROOT}/livox_ros_driver2"; do
  if [[ -f "${_d}/package.xml" ]]; then
    PATHS+=("${_d}")
    break
  fi
done

cd "${REPO_ROOT}"
colcon build --symlink-install \
  --packages-select ros2_livox_simulation dong_gazebo_sim \
  --paths "${PATHS[@]}" \
  --cmake-args \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS="${ROS_DISTRO}"

echo
echo "仿真包编译完成。启动示例:"
echo "  source ${REPO_ROOT}/install/setup.bash"
echo "  ros2 launch dong_gazebo_sim mid360_sim_slam.launch.py"
echo
echo "键盘控制（另开终端）:"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true"
