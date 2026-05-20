#!/usr/bin/env bash
set -euo pipefail

# ROS 2 Humble ament `setup.bash` / `local_setup.sh` read many optional variables
# (AMENT_TRACE_SETUP_FILES, AMENT_PYTHON_EXECUTABLE, AMENT_PREFIX_PATH, …).
# With bash nounset (-u), referencing an unset var aborts sourcing. We therefore
# disable nounset for the block that sources ROS and runs rosdep against that env.

# Drop Miniconda/Anaconda from PATH for this script so `python3` / CMake pick
# Ubuntu's Python 3.10 (Humble's rosidl / empy expect that). In an interactive shell,
# run `conda deactivate` until the prompt shows no `(base)` before colcon build.
__dong_clean_path=
IFS=:
for __p in ${PATH}; do
  case "${__p}" in
    *miniconda*|*Miniconda*|*anaconda*|*Anaconda*) ;;
    *) __dong_clean_path="${__dong_clean_path:+$__dong_clean_path:}${__p}" ;;
  esac
done
unset IFS __p
export PATH="/usr/bin:/usr/local/bin:${__dong_clean_path}"
unset __dong_clean_path
PYTHON_FOR_PIP="/usr/bin/python3"

# Install system, ROS 2 Humble, and Python dependencies needed by the three
# packages in this directory: dong_map_msgs, dong_octomap, and octo_planner.
#
# Target OS: Ubuntu 22.04 (jammy)
# Usage:
#   cd /path/to/ros2_ws/src/dong_3d_nav
#   bash install_deps_humble.sh
#
# Notes:
# - This script installs external dependencies only. Local workspace packages
#   such as go2_description (URDF), fast_lio / livox_ros_driver2 (Mid-360 + FAST-LIO;
#   clone into this repo root with scripts/sync_fastlio_mid360_deps.sh),
#   unitree_ros2 (clone separately), jie_ware_ros2, sllidar_ros2 still need to be
#   present in the same ROS 2 workspace if you want the full navigation stack.
# - dong_octomap: PCD→OctoMap (`pcd_to_octomap_node`) uses a small in-tree PCD reader + zlib;
#   Open3D C++ is optional and only needed for `octomap_open3d_viewer_node`. The script can
#   still install the official Open3D devel tarball + libc++ for that viewer.
# - If you use Miniconda: deactivate it (`conda deactivate` until no env) before
#   `colcon build`, or CMake may pick conda Python and ROSIDL will fail on Humble.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
UBUNTU_CODENAME="$(. /etc/os-release && echo "${VERSION_CODENAME}")"

# Open3D official C++ devel tarball (fallback when apt has no libopen3d-dev).
OPEN3D_DEVEL_VERSION="0.17.0"
OPEN3D_DEVEL_DIR="${HOME}/.local/opt/open3d-${OPEN3D_DEVEL_VERSION}-cxx11"
OPEN3D_DEVEL_URL="https://github.com/isl-org/Open3D/releases/download/v${OPEN3D_DEVEL_VERSION}/open3d-devel-linux-x86_64-cxx11-abi-${OPEN3D_DEVEL_VERSION}.tar.xz"

if [[ "${UBUNTU_CODENAME}" != "jammy" ]]; then
  echo "ERROR: this script targets Ubuntu 22.04 (jammy), current codename: ${UBUNTU_CODENAME}" >&2
  exit 1
fi

if [[ "${ROS_DISTRO}" != "humble" ]]; then
  echo "ERROR: this script targets ROS 2 Humble on Ubuntu 22.04, ROS_DISTRO=${ROS_DISTRO}" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  curl \
  gnupg \
  lsb-release \
  software-properties-common

if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
  sudo add-apt-repository universe -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  python3-vtk9 \
  python3-numpy \
  python3-pil \
  python3-pyqt5 \
  python3-yaml \
  libeigen3-dev \
  liboctomap-dev \
  libopencv-dev \
  zlib1g-dev \
  libqt5gui5 \
  libqt5opengl5 \
  libqt5widgets5 \
  libqt5x11extras5 \
  libtinyxml2-dev \
  libvtk9-dev \
  qt5-qmake \
  qtbase5-dev \
  qtbase5-dev-tools \
  ros-${ROS_DISTRO}-ament-cmake \
  ros-${ROS_DISTRO}-ament-cmake-python \
  ros-${ROS_DISTRO}-builtin-interfaces \
  ros-${ROS_DISTRO}-geometry-msgs \
  ros-${ROS_DISTRO}-launch \
  ros-${ROS_DISTRO}-launch-ros \
  ros-${ROS_DISTRO}-nav-msgs \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-nav2-lifecycle-manager \
  ros-${ROS_DISTRO}-nav2-map-server \
  ros-${ROS_DISTRO}-octomap \
  ros-${ROS_DISTRO}-octomap-msgs \
  ros-${ROS_DISTRO}-octomap-ros \
  ros-${ROS_DISTRO}-octomap-rviz-plugins \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-rosbridge-server \
  ros-${ROS_DISTRO}-rosidl-default-generators \
  ros-${ROS_DISTRO}-rosidl-default-runtime \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-sensor-msgs \
  ros-${ROS_DISTRO}-sensor-msgs-py \
  ros-${ROS_DISTRO}-std-msgs \
  ros-${ROS_DISTRO}-tf2 \
  ros-${ROS_DISTRO}-tf2-geometry-msgs \
  ros-${ROS_DISTRO}-tf2-msgs \
  ros-${ROS_DISTRO}-tf2-ros \
  ros-${ROS_DISTRO}-visualization-msgs

# Prebuilt libOpen3D.so needs LLVM libc++ at link/runtime (NEEDED: libc++.so.1).
# 勿再 apt install libopen3d-dev：其与 libc++-dev 冲突（libopen3d 拉取 libgoogle-glog-dev
# 需要 libunwind-dev，而 libc++ 使用 libunwind-14-dev，apt 会报「无法安装 libunwind-dev」）。
# Open3D C++ 请使用下方官方 tarball（~/.local/opt/open3d-*），不依赖 apt 的 libopen3d-dev。
sudo apt-get install -y libc++-dev libc++abi-dev

# Official Open3D C++ devel (Open3DConfig.cmake + headers + libOpen3D.so), user-local.
# curl 不走 shell 里的 ALL_PROXY：代理未启动时会出现「连接 127.0.0.1:7897 被拒绝」。
if [[ "$(uname -m)" == "x86_64" ]] && [[ ! -f "${OPEN3D_DEVEL_DIR}/lib/cmake/Open3D/Open3DConfig.cmake" ]]; then
  echo "Downloading Open3D C++ devel ${OPEN3D_DEVEL_VERSION} -> ${OPEN3D_DEVEL_DIR}"
  mkdir -p "${OPEN3D_DEVEL_DIR}"
  o3d_tmp="$(mktemp)"
  if env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY \
       -u all_proxy -u ALL_PROXY -u ftp_proxy -u FTP_PROXY \
       curl -fsSL --retry 3 --connect-timeout 60 -o "${o3d_tmp}" "${OPEN3D_DEVEL_URL}"; then
    tar -xJf "${o3d_tmp}" -C "${OPEN3D_DEVEL_DIR}" --strip-components=1
    rm -f "${o3d_tmp}"
  else
    rm -f "${o3d_tmp}"
    echo "WARN: Open3D C++ devel 下载失败（已跳过，不影响其余 apt/rosdep 依赖）。" >&2
    echo "  常见原因：环境变量 ALL_PROXY 指向 127.0.0.1:7897 等本地代理，但代理未运行。" >&2
    echo "  处理：启动代理后重跑本脚本，或临时 unset ALL_PROXY all_proxy 再执行。" >&2
    echo "  手动安装:" >&2
    echo "    mkdir -p ${OPEN3D_DEVEL_DIR}" >&2
    echo "    curl -fsSL -o /tmp/open3d-devel.tar.xz ${OPEN3D_DEVEL_URL}" >&2
    echo "    tar -xJf /tmp/open3d-devel.tar.xz -C ${OPEN3D_DEVEL_DIR} --strip-components=1" >&2
  fi
fi

if [[ -f "${OPEN3D_DEVEL_DIR}/lib/cmake/Open3D/Open3DConfig.cmake" ]]; then
  echo "Open3D C++ devel installed at: ${OPEN3D_DEVEL_DIR}"
else
  echo "NOTE: Open3D C++ devel tarball is x86_64-only; on other arches install Open3D C++ yourself." >&2
fi

"${PYTHON_FOR_PIP}" -m pip install --user --upgrade pip
"${PYTHON_FOR_PIP}" -m pip install --user --upgrade open3d

if [[ ! -d /etc/ros/rosdep/sources.list.d ]]; then
  sudo rosdep init || true
fi
rosdep update

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"

ROSDEP_SKIP_KEYS=(
  fast_lio
  go2_description
  jie_ware_ros2
  livox_ros_driver2
  opencv
  sllidar_ros2
)

rosdep install \
  --from-paths "${SCRIPT_DIR}/dong_map_msgs" "${SCRIPT_DIR}/dong_octomap" "${SCRIPT_DIR}/octo_planner" \
  --ignore-src \
  --rosdistro "${ROS_DISTRO}" \
  --skip-keys "${ROSDEP_SKIP_KEYS[*]}" \
  -r -y

set -u

# Colcon workspace root: first ancestor of this repo that has a top-level src/ (…/ws/src/…),
# else this repo directory (packages live directly under SCRIPT_DIR).
COLCON_WS_ROOT="$(cd "${SCRIPT_DIR}" && pwd)"
d="${COLCON_WS_ROOT}"
while [[ "${d}" != "/" ]]; do
  if [[ -d "${d}/src" ]]; then
    COLCON_WS_ROOT="${d}"
    break
  fi
  d="$(dirname "${d}")"
done

cat <<EOF

Dependency installation finished.

Next steps:
  1. Ensure these local/non-apt packages also exist in your workspace if needed:
     go2_description fast_lio livox_ros_driver2 jie_ware_ros2 sllidar_ros2
  2. Open3D C++: this script installs headers/libs under ${OPEN3D_DEVEL_DIR} (x86_64).
     Build with that prefix on CMAKE_PREFIX_PATH, e.g.:
       export CMAKE_PREFIX_PATH="${OPEN3D_DEVEL_DIR}:\${CMAKE_PREFIX_PATH:-}"
  3. Build from the workspace root, for example (exit Miniconda / \`conda deactivate\` first if you use it):
     cd "${COLCON_WS_ROOT}"
     # If your shell uses \`set -u\`, wrap ROS setup (ament touches unset opt vars):
     set +u
     source /opt/ros/${ROS_DISTRO}/setup.bash
     set -u
     export CMAKE_PREFIX_PATH="${OPEN3D_DEVEL_DIR}:\${CMAKE_PREFIX_PATH:-}"
     colcon build --packages-select dong_map_msgs dong_octomap octo_planner --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
EOF
