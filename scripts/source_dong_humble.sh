#!/usr/bin/env bash
# 干净加载 Humble + 本工作区，避免与 Noetic 混用导致 bad_alloc / librclcpp not found。
# 用法: source /path/to/navmid360/scripts/source_dong_humble.sh
#
# 注意：勿 source install/setup.bash —— 该文件在 colcon 生成时若曾加载 Noetic，
# 会把 /opt/ros/noetic 链进环境，Fast-DDS 会反复打印 bad_alloc caught。

DONG_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

_dong_strip_ros1_paths() {
  local _name="$1"
  local _val="${!_name:-}"
  [[ -z "${_val}" ]] && return 0
  _val="$(echo ":${_val}:" | tr ':' '\n' | grep -v '/opt/ros/noetic' \
    | grep -v 'robot-control/devel' | grep -v '/opt/MVS' | grep -v '^$' | paste -sd: -)"
  _val="${_val#:}"
  _val="${_val%:}"
  export "${_name}=${_val}"
}

# 去掉 Noetic / 其它 ROS 常见变量（勿 unset 整个 LD_LIBRARY_PATH，只滤 ROS 路径）
for _var in ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH \
  CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH; do
  unset "${_var}" 2>/dev/null || true
done
_dong_strip_ros1_paths LD_LIBRARY_PATH
_dong_strip_ros1_paths PATH
_dong_strip_ros1_paths CMAKE_PREFIX_PATH
_dong_strip_ros1_paths AMENT_PREFIX_PATH

source /opt/ros/humble/setup.bash
# 只叠加本工作区包，不 source install/setup.bash（避免 colcon 前缀链里的 Noetic）
if [[ -f "${DONG_ROOT}/install/local_setup.bash" ]]; then
  source "${DONG_ROOT}/install/local_setup.bash"
fi
_dong_strip_ros1_paths LD_LIBRARY_PATH
_dong_strip_ros1_paths PATH
_dong_strip_ros1_paths CMAKE_PREFIX_PATH
_dong_strip_ros1_paths AMENT_PREFIX_PATH
unset -f _dong_strip_ros1_paths

# 系统 libusb 优先（~/.bashrc 里海康 MVS 的 /opt/MVS/lib/64/libusb 过旧，会导致 PCL 报 libusb_set_option）
case ":${LD_LIBRARY_PATH:-}:" in
  *":/lib/x86_64-linux-gnu:"*) ;;
  *) export LD_LIBRARY_PATH="/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" ;;
esac

# 保证 Humble 库优先（避免 ~/.bashrc 里 Noetic 的 LD_LIBRARY_PATH 覆盖）
HUMBLE_LIB="/opt/ros/humble/lib"
HUMBLE_LIB_ARCH="/opt/ros/humble/lib/x86_64-linux-gnu"
DONG_LIB="${DONG_ROOT}/install/lib"
for _p in "${HUMBLE_LIB_ARCH}" "${HUMBLE_LIB}" "${DONG_LIB}"; do
  if [[ -d "${_p}" ]]; then
    case ":${LD_LIBRARY_PATH:-}:" in
      *":${_p}:"*) ;;
      *) export LD_LIBRARY_PATH="${_p}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" ;;
    esac
  fi
done
unset _p HUMBLE_LIB HUMBLE_LIB_ARCH DONG_LIB

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ROS2 用 AMENT，勿保留 Noetic 的 ROS_PACKAGE_PATH（否则 rviz2 报 rviz_common not found 并崩溃）
unset ROS_PACKAGE_PATH 2>/dev/null || true

if ldd "${DONG_ROOT}/install/icp_relocalization/lib/icp_relocalization/transform_publisher" 2>/dev/null \
  | grep -q '/opt/ros/noetic'; then
  echo "WARN: icp_relocalization 仍链接 Noetic 库，请在本终端仅 source 本脚本后 colcon build icp_relocalization fast_lio" >&2
fi

echo "OK: ROS_DISTRO=${ROS_DISTRO}, workspace=${DONG_ROOT}/install"
