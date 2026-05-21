#!/usr/bin/env bash
# Go2 导航栈一键编译（Humble）
#
# 编译顺序（必须遵守）:
#   1. livox_ros_driver2
#   2. fast_lio + icp_relocalization（Fast-LIO2-Localization）
#   3. dong_map_msgs / dong_relocalization / dong_octomap / go2_nav_stack / octo_planner
#
# 用法:
#   bash scripts/build_all_humble.sh              # 完整编译
#   bash scripts/build_all_humble.sh --livox-only
#   bash scripts/build_all_humble.sh --rest-only
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

LIVOX_ONLY=false
REST_ONLY=false

for arg in "$@"; do
  case "${arg}" in
    --livox-only) LIVOX_ONLY=true ;;
    --rest-only) REST_ONLY=true ;;
    -h|--help)
      cat <<'EOF'
用法: bash scripts/build_all_humble.sh [选项]

选项:
  (无)           先 Livox，再 fast_lio / icp / 核心包
  --livox-only   仅编译 livox_ros_driver2
  --rest-only    编译 fast_lio + icp + 核心包（须已编 Livox）

里程计: FAST-LIO2（不再编译 kiss_icp）
EOF
      exit 0
      ;;
    *) echo "未知参数: ${arg}" >&2; exit 1 ;;
  esac
done

if [[ "${LIVOX_ONLY}" == true && "${REST_ONLY}" == true ]]; then
  echo "错误: --livox-only 与 --rest-only 不能同时使用" >&2
  exit 1
fi

# shellcheck source=scripts/dong_ros_env.sh
source "${REPO_ROOT}/scripts/dong_ros_env.sh"
dong_source_humble || exit 1

CMAKE_COMMON=(
  -DPYTHON_EXECUTABLE=/usr/bin/python3
  -DPython3_EXECUTABLE=/usr/bin/python3
)

assert_livox_installed() {
  local livox_install="${REPO_ROOT}/install/livox_ros_driver2/share/livox_ros_driver2/package.xml"
  if [[ ! -f "${livox_install}" ]]; then
    echo "错误: livox_ros_driver2 尚未编译。" >&2
    echo "  bash scripts/build_all_humble.sh --livox-only && source install/setup.bash" >&2
    exit 1
  fi
}

assert_fastlio_installed() {
  if [[ ! -f "${REPO_ROOT}/install/fast_lio/share/fast_lio/package.xml" ]]; then
    echo "错误: fast_lio 尚未编译。" >&2
    echo "  请确认 Fast-LIO2-Localization 存在: bash scripts/sync_fastlio2_localization.sh" >&2
    exit 1
  fi
}

build_livox() {
  echo "=========================================="
  echo " [第 1 步] 编译 livox_ros_driver2"
  echo "=========================================="
  bash "${REPO_ROOT}/scripts/build_livox_ros_driver2.sh" "${REPO_ROOT}"
  assert_livox_installed
  # shellcheck source=/dev/null
  set +u
  source "${REPO_ROOT}/install/setup.bash"
  set -u
}

build_rest() {
  assert_livox_installed
  # shellcheck source=/dev/null
  set +u
  source "${REPO_ROOT}/install/setup.bash"
  set -u

  if [[ ! -f "${REPO_ROOT}/Fast-LIO2-Localization/FAST_LIO/package.xml" ]]; then
    echo ">>> 同步 Fast-LIO2-Localization..."
    bash "${REPO_ROOT}/scripts/sync_fastlio2_localization.sh"
  fi

  echo ">>> [2/4] 编译 fast_lio..."
  bash "${REPO_ROOT}/scripts/build_fast_lio_polaris_humble.sh"
  # shellcheck source=/dev/null
  set +u
  source "${REPO_ROOT}/install/setup.bash"
  set -u
  assert_fastlio_installed

  echo ">>> [3/4] 编译 icp_relocalization..."
  bash "${REPO_ROOT}/scripts/build_icp_relocalization_humble.sh"
  # shellcheck source=/dev/null
  set +u
  source "${REPO_ROOT}/install/setup.bash"
  set -u

  echo ">>> [4/4] 编译本仓库核心包..."
  dong_prepend_open3d_cmake
  for _pkg in dong_map_msgs dong_relocalization dong_octomap go2_nav_stack octo_planner; do
    rm -rf "${REPO_ROOT}/build/${_pkg}" "${REPO_ROOT}/install/${_pkg}"
  done
  unset _pkg

  colcon build --symlink-install \
    --packages-select dong_map_msgs dong_relocalization dong_octomap go2_nav_stack octo_planner \
    --cmake-args "${CMAKE_COMMON[@]}"
}

if [[ "${REST_ONLY}" == true ]]; then
  build_rest
elif [[ "${LIVOX_ONLY}" == true ]]; then
  build_livox
else
  build_livox
  echo
  build_rest
fi

echo
echo "=========================================="
echo " 编译完成（FAST-LIO2，无 kiss_icp）"
echo " source ${REPO_ROOT}/install/setup.bash"
echo " ./scripts/go2_nav_oneclick.sh"
echo "=========================================="
