#!/usr/bin/env bash
# 在「仅 Foxy」环境下编译 Fast-LIO2-Localization/icp_relocalization。
# 若 shell 里同时 source 了 Noetic（CMAKE_PREFIX_PATH 含 /opt/ros/noetic），会误用 ROS1 头文件导致编译失败。
#
# 用法:
#   cd /path/to/dong_3d_nav
#   bash scripts/build_icp_relocalization_foxy.sh
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ICP_DIR="${REPO_ROOT}/Fast-LIO2-Localization/icp_relocalization"

if [[ ! -f "${ICP_DIR}/package.xml" ]]; then
  echo "未找到 ${ICP_DIR}，请先: bash scripts/sync_fastlio2_localization.sh" >&2
  exit 1
fi

bash "${REPO_ROOT}/scripts/patch_icp_relocalization_foxy.sh" "${ICP_DIR}"

# 干净环境：只保留 Foxy + 本工作区 livox（若已 install）
exec env -i \
  HOME="${HOME}" \
  USER="${USER:-$(id -un)}" \
  LANG="${LANG:-en_US.UTF-8}" \
  PATH="/usr/bin:/bin:/usr/local/bin:/opt/ros/foxy/bin" \
  bash -lc "
    set -e
    source /opt/ros/foxy/setup.bash
  LIVOX_INSTALL=\"${REPO_ROOT}/install/livox_ros_driver2\"
  CMAKE_EXTRA=\"/opt/ros/foxy\"
  if [[ -f \"\${LIVOX_INSTALL}/share/livox_ros_driver2/package.xml\" ]]; then
    source \"\${LIVOX_INSTALL}/../setup.bash\" 2>/dev/null || true
    CMAKE_EXTRA=\"\${CMAKE_EXTRA}:\${LIVOX_INSTALL}\"
  else
    echo \"警告: 未找到已编译的 livox_ros_driver2，请先 colcon build livox_ros_driver2\" >&2
  fi
  cd \"${REPO_ROOT}\"
  colcon build --symlink-install \\
    --packages-select icp_relocalization \\
    --paths \"${ICP_DIR}\" \\
    --cmake-args \\
      -DPYTHON_EXECUTABLE=/usr/bin/python3 \\
      -DCMAKE_PREFIX_PATH=\"\${CMAKE_EXTRA}\"
  echo
  echo \"完成。请在本终端执行: source ${REPO_ROOT}/install/setup.bash\"
"
