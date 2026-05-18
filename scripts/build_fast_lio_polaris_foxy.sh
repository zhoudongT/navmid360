#!/usr/bin/env bash
# 在「仅 Foxy」环境下编译 Fast-LIO2-Localization/FAST_LIO（包名 fast_lio）。
# 若 shell 同时 source 了 Noetic，会误链 ROS1 的 pcl_conversions 导致编译失败。
#
# 用法:
#   cd /path/to/dong_3d_nav
#   bash scripts/build_fast_lio_polaris_foxy.sh
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAST_LIO_DIR="${REPO_ROOT}/Fast-LIO2-Localization/FAST_LIO"

if [[ ! -f "${FAST_LIO_DIR}/package.xml" ]]; then
  echo "未找到 ${FAST_LIO_DIR}，请先: bash scripts/sync_fastlio2_localization.sh" >&2
  exit 1
fi

bash "${REPO_ROOT}/scripts/patch_fast_lio_polaris_foxy.sh" "${FAST_LIO_DIR}" 2>/dev/null || true
python3 "${REPO_ROOT}/scripts/patch_fast_lio_ros2_cmake.py" "${FAST_LIO_DIR}" 2>/dev/null || true

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
      CMAKE_EXTRA=\"\${CMAKE_EXTRA}:\${LIVOX_INSTALL}\"
    else
      echo \"警告: 未找到已编译的 livox_ros_driver2，请先 colcon build livox_ros_driver2\" >&2
    fi
    cd \"${REPO_ROOT}\"
    colcon build --symlink-install \\
      --packages-select fast_lio \\
      --paths \"${FAST_LIO_DIR}\" \\
      --cmake-args \\
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \\
        -DCMAKE_PREFIX_PATH=\"\${CMAKE_EXTRA}\"
    echo
    echo \"完成。请执行:\"
    echo \"  source /opt/ros/foxy/setup.bash\"
    echo \"  source ${REPO_ROOT}/install/setup.bash\"
    echo \"  ros2 pkg prefix fast_lio\"
"
