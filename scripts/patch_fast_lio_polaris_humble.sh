#!/usr/bin/env bash
# Polaris Fast-LIO2-Localization/FAST_LIO 在 Humble 下的源码补丁（可重复执行）：
# - pcl_ros::transformPointCloud → pcl::transformPointCloud + Eigen
# - tf2_geometry_msgs.h → tf2_geometry_msgs.hpp（Humble）
# - CMake/package.xml 去掉 pcl_ros 依赖
#
# 用法:
#   bash scripts/patch_fast_lio_polaris_humble.sh
#   bash scripts/patch_fast_lio_polaris_humble.sh /path/to/FAST_LIO
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAST_LIO_DIR="${1:-${REPO_ROOT}/Fast-LIO2-Localization/FAST_LIO}"
LASER_CPP="${FAST_LIO_DIR}/src/laserMapping.cpp"
CMAKE_FILE="${FAST_LIO_DIR}/CMakeLists.txt"
PKG_XML="${FAST_LIO_DIR}/package.xml"

if [[ ! -f "${LASER_CPP}" ]]; then
  echo "未找到 ${LASER_CPP}" >&2
  exit 1
fi

if grep -q 'pcl_ros/transforms.hpp' "${LASER_CPP}"; then
  sed -i 's|#include <pcl_ros/transforms.hpp>|#include <pcl/common/transforms.h>\n#include <Eigen/Geometry>|' "${LASER_CPP}"
  echo "已替换 laserMapping.cpp 中的 pcl_ros/transforms.hpp"
fi

if grep -q 'tf2_geometry_msgs/tf2_geometry_msgs.hpppp' "${LASER_CPP}"; then
  sed -i 's|tf2_geometry_msgs/tf2_geometry_msgs\.hpppp|tf2_geometry_msgs/tf2_geometry_msgs.hpp|g' "${LASER_CPP}"
  echo "已修正 laserMapping.cpp 中重复的 hpp 后缀"
elif grep -qE 'tf2_geometry_msgs/tf2_geometry_msgs\.h([^p]|")' "${LASER_CPP}"; then
  sed -i 's|tf2_geometry_msgs/tf2_geometry_msgs\.h"|tf2_geometry_msgs/tf2_geometry_msgs.hpp"|g' "${LASER_CPP}"
  echo "已修正 tf2_geometry_msgs 头文件（Humble）"
fi

if grep -q 'pcl_ros::transformPointCloud' "${LASER_CPP}"; then
  python3 - "${LASER_CPP}" <<'PY'
import re, sys
path = sys.argv[1]
text = open(path, encoding="utf-8").read()
old = """                    pcl_ros::transformPointCloud(*feats_down_prior_map, *feats_down_prior_map, trans.inverse());"""
new = """                    const tf2::Transform trans_inv = trans.inverse();
                    Eigen::Affine3f T = Eigen::Affine3f::Identity();
                    T.translation() << static_cast<float>(trans_inv.getOrigin().x()),
                        static_cast<float>(trans_inv.getOrigin().y()),
                        static_cast<float>(trans_inv.getOrigin().z());
                    const tf2::Quaternion q = trans_inv.getRotation();
                    Eigen::Quaternionf eq(static_cast<float>(q.w()), static_cast<float>(q.x()),
                        static_cast<float>(q.y()), static_cast<float>(q.z()));
                    T.linear() = eq.toRotationMatrix();
                    pcl::transformPointCloud(*feats_down_prior_map, *feats_down_prior_map, T);"""
if old not in text:
    sys.exit(0)
open(path, "w", encoding="utf-8").write(text.replace(old, new, 1))
print("已替换 laserMapping.cpp 中的 pcl_ros::transformPointCloud")
PY
fi

if [[ -f "${CMAKE_FILE}" ]] && grep -q 'find_package(pcl_ros REQUIRED)' "${CMAKE_FILE}"; then
  sed -i '/find_package(pcl_ros REQUIRED)/d' "${CMAKE_FILE}"
  sed -i '/^[[:space:]]*pcl_ros$/d' "${CMAKE_FILE}"
  echo "已从 CMakeLists.txt 移除 pcl_ros"
fi

if [[ -f "${PKG_XML}" ]] && grep -q '<depend>pcl_ros</depend>' "${PKG_XML}"; then
  sed -i '/<depend>pcl_ros<\/depend>/d' "${PKG_XML}"
  echo "已从 package.xml 移除 pcl_ros"
fi

if grep -q 'std_srvs::srv::Trigger::Request> req' "${LASER_CPP}" && \
   ! grep -q 'Trigger::Request::ConstSharedPtr' "${LASER_CPP}"; then
  sed -i 's/const std::shared_ptr<std_srvs::srv::Trigger::Request> req/std_srvs::srv::Trigger::Request::ConstSharedPtr req/' "${LASER_CPP}"
  sed -i 's/std::shared_ptr<std_srvs::srv::Trigger::Response> res/std_srvs::srv::Trigger::Response::SharedPtr res/' "${LASER_CPP}"
  echo "已修正 map_save_callback 为 Humble service 签名"
fi

echo "Polaris fast_lio Humble 补丁检查完成。编译: bash scripts/build_fast_lio_polaris_humble.sh"
