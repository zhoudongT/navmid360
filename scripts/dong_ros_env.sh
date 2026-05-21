#!/usr/bin/env bash
# 供 build_*.sh 引用：清理陈旧工作区 overlay，仅加载 ROS 2 Humble。
# 用法: source "$(dirname "$0")/dong_ros_env.sh"  或  source scripts/dong_ros_env.sh

dong_clean_path_for_build() {
  local __dong_clean_path=
  local __p
  IFS=:
  for __p in ${PATH}; do
    case "${__p}" in
      *miniconda*|*Miniconda*|*anaconda*|*Anaconda*) ;;
      *) __dong_clean_path="${__dong_clean_path:+$__dong_clean_path:}${__p}" ;;
    esac
  done
  unset IFS __p
  export PATH="/usr/bin:/usr/local/bin:/opt/ros/humble/bin:${__dong_clean_path}"
  unset __dong_clean_path
}

dong_reset_ros_overlay() {
  # rm -rf install 后若仍 source 过旧 install/setup.bash，colcon 会报大量 prefix 不存在
  unset AMENT_PREFIX_PATH
  unset CMAKE_PREFIX_PATH
  unset COLCON_PREFIX_PATH
  unset ROS_PACKAGE_PATH
  unset PYTHONPATH
}

dong_source_humble() {
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "错误: 未找到 /opt/ros/humble/setup.bash" >&2
    return 1
  fi
  dong_reset_ros_overlay
  dong_clean_path_for_build
  # shellcheck source=/dev/null
  set +u
  source /opt/ros/humble/setup.bash
  set -u
}

dong_prepend_open3d_cmake() {
  local open3d_devel="${HOME}/.local/opt/open3d-0.17.0-cxx11"
  if [[ -f "${open3d_devel}/lib/cmake/Open3D/Open3DConfig.cmake" ]]; then
    export CMAKE_PREFIX_PATH="${open3d_devel}:${CMAKE_PREFIX_PATH:-}"
  fi
}
