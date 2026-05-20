#!/usr/bin/env bash
# Polaris icp_relocalization 在 Humble 下的构建补丁：
# - 移除未使用的 octomap_ros
# - 用 pcl_conversions + PCL 替代 pcl_ros（未装 ros-humble-pcl-ros 时也能编）
# - 默认不编 sac_ia_gicp（example.launch 未使用）
#
# 用法:
#   bash scripts/patch_icp_relocalization_humble.sh [icp_relocalization目录]
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ICP_DIR="${1:-${REPO_ROOT}/Fast-LIO2-Localization/icp_relocalization}"
ICP_CMAKE="${ICP_DIR}/CMakeLists.txt"
ICP_PKG="${ICP_DIR}/package.xml"

ICP_CPP="${ICP_DIR}/src/icp_node.cpp"

if [[ ! -f "${ICP_CMAKE}" ]]; then
  echo "未找到 ${ICP_CMAKE}，请先: bash scripts/sync_fastlio2_localization.sh" >&2
  exit 1
fi

if [[ -f "${ICP_CPP}" ]]; then
  if grep -q 'tf2_geometry_msgs/tf2_geometry_msgs.hpppp' "${ICP_CPP}"; then
    sed -i 's|tf2_geometry_msgs/tf2_geometry_msgs\.hpppp|tf2_geometry_msgs/tf2_geometry_msgs.hpp|g' "${ICP_CPP}"
    echo "已修正 icp_node.cpp 中重复的 hpp 后缀。"
  elif grep -qE 'tf2_geometry_msgs/tf2_geometry_msgs\.h([^p]|")' "${ICP_CPP}"; then
    sed -i 's|tf2_geometry_msgs/tf2_geometry_msgs\.h"|tf2_geometry_msgs/tf2_geometry_msgs.hpp"|g' "${ICP_CPP}"
    echo "已修正 icp_node.cpp 中 tf2_geometry_msgs 头文件（Humble）。"
  fi
fi

if grep -q 'find_package(octomap_ros REQUIRED)' "${ICP_CMAKE}"; then
  sed -i '/find_package(octomap_ros REQUIRED)/d' "${ICP_CMAKE}"
  echo "已移除 octomap_ros。"
fi

if grep -q 'find_package(pcl_ros REQUIRED)' "${ICP_CMAKE}"; then
  sed -i 's/find_package(pcl_ros REQUIRED)/find_package(pcl_conversions REQUIRED)\nfind_package(PCL REQUIRED COMPONENTS common io filters registration search kdtree)/' "${ICP_CMAKE}"
  sed -i 's/pcl_ros/pcl_conversions/g' "${ICP_CMAKE}"
  if ! grep -q 'PCL_INCLUDE_DIRS' "${ICP_CMAKE}"; then
    sed -i '/include_directories(\${EIGEN3_INCLUDE_DIR})/a\
include_directories(${PCL_INCLUDE_DIRS})\
link_directories(${PCL_LIBRARY_DIRS})\
add_definitions(${PCL_DEFINITIONS})' "${ICP_CMAKE}"
    sed -i '/^add_executable(icp_node/a\
target_link_libraries(icp_node ${PCL_LIBRARIES})' "${ICP_CMAKE}"
  fi
  sed -i 's/ament_target_dependencies(transform_publisher rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs pcl_conversions)/ament_target_dependencies(transform_publisher rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)/' "${ICP_CMAKE}"
  echo "已将 pcl_ros 替换为 pcl_conversions + PCL。"
fi

if [[ -f "${ICP_PKG}" ]] && grep -q '<build_depend>pcl_ros</build_depend>' "${ICP_PKG}"; then
  sed -i 's/<build_depend>pcl_ros<\/build_depend>/<build_depend>pcl_conversions<\/build_depend>\n  <build_depend>libpcl-all-dev<\/build_depend>/' "${ICP_PKG}"
  echo "已更新 package.xml 依赖。"
fi

if [[ -f "${ICP_PKG}" ]] && ! grep -q 'livox_ros_driver2' "${ICP_PKG}"; then
  sed -i '/<build_depend>libpcl-all-dev<\/build_depend>/a\
  <build_depend>livox_ros_driver2</build_depend>\
  <exec_depend>livox_ros_driver2</exec_depend>' "${ICP_PKG}"
  echo "已在 package.xml 加入 livox_ros_driver2 依赖（保证 colcon 先编 Livox）。"
fi

# example.launch 未使用 sac_ia_gicp；跳过可加快编译并避免额外 PCL 链接问题
if grep -q 'sac_ia_gicp' "${ICP_CMAKE}"; then
  sed -i '/^add_executable(sac_ia_gicp/,/^)$/d' "${ICP_CMAKE}"
  sed -i '/^ament_target_dependencies(sac_ia_gicp/d' "${ICP_CMAKE}"
  sed -i 's/TARGETS icp_node transform_publisher sac_ia_gicp/TARGETS icp_node transform_publisher/' "${ICP_CMAKE}"
  echo "已跳过 sac_ia_gicp 目标（上游 example 未使用）。"
fi

echo "当前 ${ICP_CMAKE} 已按 Humble 维护。编译: bash scripts/build_icp_relocalization_humble.sh"
