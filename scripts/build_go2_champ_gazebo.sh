#!/usr/bin/env bash
# 编译 go2_nav2_ros2 CHAMP Go2 腿控仿真 + go2_description（含 Mid-360 xacro）
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${REPO_ROOT}/third_party/go2_champ_sim"

if [[ ! -f "${DEST}/.synced_at" ]]; then
  echo "请先: bash scripts/sync_go2_champ_gazebo.sh" >&2
  exit 1
fi

set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

BUILD_PATHS=(
  "${REPO_ROOT}/go2_description"
)
for pkg_dir in "${DEST}"/champ/champ*; do
  [[ -f "${pkg_dir}/package.xml" ]] && BUILD_PATHS+=("${pkg_dir}")
done
if [[ -d "${DEST}/aws_robomaker_small_warehouse_world" ]]; then
  BUILD_PATHS+=("${DEST}/aws_robomaker_small_warehouse_world")
fi

echo "[build-go2-champ] 编译 ${#BUILD_PATHS[@]} 个包（champ_config / champ_gazebo / go2_description …）…"
cd "${REPO_ROOT}"
colcon build --symlink-install \
  --paths "${BUILD_PATHS[@]}" \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3

echo
echo "完成。请执行:"
echo "  source ${REPO_ROOT}/scripts/source_navmid360.sh"
echo "  ros2 pkg prefix champ_config"
