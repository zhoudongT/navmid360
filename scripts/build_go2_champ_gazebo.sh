#!/usr/bin/env bash
# 编译 CHAMP Go2 腿控仿真 + go2_description（含 Mid-360 xacro）
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

# colcon --paths 只扫描一层；champ 各包在 champ/champ_* 子目录，需逐个列出
BUILD_PATHS=(
  "${REPO_ROOT}/go2_description"
  "${DEST}/go2_config"
  "${DEST}/champ_teleop"
)
for pkg_dir in "${DEST}"/champ/champ*; do
  [[ -f "${pkg_dir}/package.xml" ]] && BUILD_PATHS+=("${pkg_dir}")
done

echo "[build-go2-champ] 编译 ${#BUILD_PATHS[@]} 个包路径（champ_bringup / champ_gazebo / go2_config …）…"
cd "${REPO_ROOT}"
colcon build --symlink-install \
  --paths "${BUILD_PATHS[@]}" \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3

echo
echo "完成。请执行（勿在 set -u 下直接 source install/setup.bash）:"
echo "  source ${REPO_ROOT}/scripts/source_navmid360.sh"
echo "  ros2 pkg prefix champ_bringup"
