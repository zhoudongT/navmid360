#!/usr/bin/env bash
# 将 FAST_LIO_ROS2（ros2 分支）与 Livox Mid-360 驱动 livox_ros_driver2 克隆到本仓库根目录下，
# 与 octo_planner、dong_octomap 等并列；colcon 从工作区 src 扫描时会一并发现这些包。
# 目录已加入 .gitignore，不会随本仓库提交。
#
# 用法:
#   cd /path/to/dong_3d_nav
#   bash scripts/sync_fastlio_mid360_deps.sh
#
# 若仍希望克隆到其它目录（例如与 dong_3d_nav 并列的 ros2_ws/src）:
#   bash scripts/sync_fastlio_mid360_deps.sh /path/to/ros2_ws/src
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_DEST="${REPO_ROOT}"
DEST="${1:-${DEFAULT_DEST}}"

FAST_LIO_URL="https://github.com/Ericsii/FAST_LIO_ROS2.git"
FAST_LIO_BRANCH="ros2"
FAST_LIO_DIR="${DEST}/fast_lio"

LIVOX_URL="https://github.com/Livox-SDK/livox_ros_driver2.git"
LIVOX_BRANCH="master"
LIVOX_DIR="${DEST}/livox_ros_driver2"

mkdir -p "${DEST}"

clone_shallow() {
  local url="$1" branch="$2" dir="$3"
  if [[ -d "${dir}/.git" ]]; then
    echo "[sync] 已存在 ${dir}，跳过克隆（如需更新请在该目录内手动 git pull）。"
    return 0
  fi
  if [[ -e "${dir}" ]]; then
    echo "[sync] 错误: ${dir} 已存在且不是 git 仓库，请删除或换名后重试。" >&2
    exit 1
  fi
  echo "[sync] git clone ${url} (-b ${branch}) -> ${dir}"
  git clone --depth 1 -b "${branch}" "${url}" "${dir}"
}

clone_shallow "${FAST_LIO_URL}" "${FAST_LIO_BRANCH}" "${FAST_LIO_DIR}"
clone_shallow "${LIVOX_URL}" "${LIVOX_BRANCH}" "${LIVOX_DIR}"

if [[ -f "${FAST_LIO_DIR}/CMakeLists.txt" ]]; then
  echo "[sync] 对 fast_lio 应用 ROS2 CMake 补丁（发行版解析 + rosidl）…"
  python3 "${REPO_ROOT}/scripts/patch_fast_lio_ros2_cmake.py" "${FAST_LIO_DIR}" || {
    echo "[sync] 警告: CMake 补丁失败，请手动运行: bash scripts/apply_fast_lio_ros2_patches.sh" >&2
  }
fi

echo
echo "[sync] 完成。编译 Livox 时请用本仓库脚本（勿在 livox 目录执行 ./build.sh ROS2，会误 cd ../../ 扫整盘）："
echo "  source /opt/ros/\${ROS_DISTRO}/setup.bash"
echo "  bash ${REPO_ROOT}/scripts/build_livox_ros_driver2.sh /path/to/ros2_ws"
echo
echo "或在工作区根目录手动（--paths 指向本仓库内 livox 目录，避免 colcon 扫描家目录）:"
echo "  colcon build --symlink-install --packages-select livox_ros_driver2 \\"
echo "    --paths ${REPO_ROOT}/livox_ros_driver2 --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3"
echo
echo "  rosdep install --from-paths src --ignore-src -r -y   # 按需"
echo "  bash ${REPO_ROOT}/scripts/apply_fast_lio_ros2_patches.sh   # 若 fast_lio 已存在且未打补丁"
echo "  colcon build --symlink-install --packages-select fast_lio dong_relocalization dong_octomap ..."
echo
echo "Mid-360 需在 livox_ros_driver2/config 中配置 MID360_config.json（本机 IP 与雷达 SN 等），参见 Livox 官方说明。"
