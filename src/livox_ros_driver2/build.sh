#!/bin/bash

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"
readonly VERSION_HUMBLE="humble"
readonly VERSION_JAZZY="jazzy"

pushd `pwd` > /dev/null
cd "$(dirname "$0")"
SCRIPT_DIR="$(pwd)"
echo "Working Path: ${SCRIPT_DIR}"

# README: clone to [workspace]/src/livox_ros_driver2 so ../../ is the workspace.
# If this repo is not under .../src/<pkg>, build in the repo root (avoid cd ../../ -> e.g. /home).
if [ -d "${SCRIPT_DIR}/../../src" ]; then
    WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
else
    WS_ROOT="${SCRIPT_DIR}"
fi
echo "Build workspace: ${WS_ROOT}"

ROS_VERSION=""
ROS_DISTRO=""

# Set working ROS version
if [ "$1" = "ROS2" ]; then
    ROS_VERSION=${VERSION_ROS2}
    # 使用当前 shell 的 ROS_DISTRO（source /opt/ros/humble/setup.bash 后为 humble）
elif [ "$1" = "humble" ]; then
    ROS_VERSION=${VERSION_ROS2}
    ROS_DISTRO=${VERSION_HUMBLE}
elif [ "$1" = "jazzy" ]; then
    ROS_VERSION=${VERSION_ROS2}
    ROS_DISTRO=${VERSION_JAZZY}
elif [ "$1" = "ROS1" ]; then
    ROS_VERSION=${VERSION_ROS1}
else
    echo "Invalid Argument"
    exit
fi
echo "ROS version is: "$ROS_VERSION

# clear `build/` folder.
# TODO: Do not clear these folders, if the last build is based on the same ROS version.
rm -rf "${WS_ROOT}/build/"
rm -rf "${WS_ROOT}/devel/"
rm -rf "${WS_ROOT}/install/"
# clear src/CMakeLists.txt if it exists.
if [ -f ../CMakeLists.txt ]; then
    rm -f ../CMakeLists.txt
fi

# exit

# substitute the files/folders: CMakeList.txt, package.xml(s)
if [ ${ROS_VERSION} = ${VERSION_ROS1} ]; then
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f package_ROS1.xml package.xml
elif [ ${ROS_VERSION} = ${VERSION_ROS2} ]; then
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f package_ROS2.xml package.xml
    cp -rf launch_ROS2/ launch/
fi

# build
pushd `pwd` > /dev/null
if [ $ROS_VERSION = ${VERSION_ROS1} ]; then
    cd "${WS_ROOT}"
    catkin_make -DROS_EDITION=${VERSION_ROS1}
elif [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    cd "${WS_ROOT}"
    colcon build --cmake-args -DROS_EDITION=${VERSION_ROS2} -DDISTRO_ROS=${ROS_DISTRO:-}
fi
popd > /dev/null

# remove the substituted folders/files
if [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    rm -rf launch/
fi

popd > /dev/null
