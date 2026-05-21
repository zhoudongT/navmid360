#!/usr/bin/env bash
# 在 ROS 2 工作区根目录下仅编译 livox_ros_driver2（--paths 限定范围，避免全盘 colcon）。
# 必须作为工作区第一个编译的包；kiss_icp / icp_relocalization 等依赖本包，请用:
#   bash scripts/build_all_humble.sh --livox-only
# 完成后再 bash scripts/build_all_humble.sh --rest-only
#
# 用法:
#   bash scripts/build_livox_ros_driver2.sh              # 工作区默认为本仓库根目录
#   bash scripts/build_livox_ros_driver2.sh ~/ros2_ws    # 指定工作区
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/dong_ros_env.sh
source "${REPO_ROOT}/scripts/dong_ros_env.sh"

LIVOX_DIR=""
for _candidate in "${REPO_ROOT}/src/livox_ros_driver2" "${REPO_ROOT}/livox_ros_driver2"; do
  if [[ -f "${_candidate}/package.xml" ]]; then
    LIVOX_DIR="${_candidate}"
    break
  fi
done
unset _candidate

if [[ -z "${LIVOX_DIR}" ]]; then
  echo "未找到 livox_ros_driver2，请先: bash scripts/sync_fastlio_mid360_deps.sh" >&2
  exit 1
fi

WS_ROOT="${1:-${REPO_ROOT}}"
if [[ ! -d "${WS_ROOT}" ]]; then
  echo "工作区目录不存在: ${WS_ROOT}" >&2
  exit 1
fi

# 干净子 shell：避免终端里残留已删除的 install/* 路径（colcon 警告 / 异常）
exec env -i \
  HOME="${HOME}" \
  USER="${USER:-$(id -un)}" \
  LANG="${LANG:-en_US.UTF-8}" \
  PATH="/usr/bin:/bin:/usr/local/bin:/opt/ros/humble/bin" \
  bash -lc "
    set -eo pipefail
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    export ROS_DISTRO=humble
    cd \"${WS_ROOT}\"
    rm -rf \"${WS_ROOT}/build/livox_ros_driver2\" \"${WS_ROOT}/install/livox_ros_driver2\"
    echo '>>> colcon build livox_ros_driver2（请勿 Ctrl+C 中断）...'
    colcon build --symlink-install --packages-select livox_ros_driver2 \\
      --paths \"${LIVOX_DIR}\" \\
      --cmake-args \\
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \\
        -DPython3_EXECUTABLE=/usr/bin/python3 \\
        -DROS_EDITION=ROS2 \\
        -DDISTRO_ROS=humble
    echo
    echo '完成。请执行: source ${WS_ROOT}/install/setup.bash'
  "
