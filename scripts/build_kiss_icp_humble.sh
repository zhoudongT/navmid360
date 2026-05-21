#!/usr/bin/env bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KISS_DIR="${REPO_ROOT}/KISS-ICP"

if [[ ! -f "${KISS_DIR}/ros/package.xml" ]]; then
  echo "未找到 KISS-ICP/ros，请先: bash scripts/sync_kiss_icp.sh" >&2
  exit 1
fi

# shellcheck source=scripts/dong_ros_env.sh
source "${REPO_ROOT}/scripts/dong_ros_env.sh"

bash "${REPO_ROOT}/scripts/patch_kiss_icp_humble.sh"

SOPHUS_CACHE="${KISS_DIR}/.cache/sophus-1.22.10.tar.gz"
USE_SYSTEM_SOPHUS=ON
if ! dpkg -s ros-humble-sophus &>/dev/null; then
  echo "[build-kiss-icp] 未安装 ros-humble-sophus，尝试 apt 安装（推荐）…" >&2
  if command -v sudo &>/dev/null && sudo -n true 2>/dev/null; then
    sudo apt-get update -qq
    sudo apt-get install -y ros-humble-sophus || true
  fi
  if ! dpkg -s ros-humble-sophus &>/dev/null; then
    if [[ -f "${SOPHUS_CACHE}" ]]; then
      echo "[build-kiss-icp] 使用本地 Sophus 缓存编译（USE_SYSTEM_SOPHUS=OFF）" >&2
      USE_SYSTEM_SOPHUS=OFF
    else
      echo "错误: 需要 ros-humble-sophus 或 Sophus 缓存。请执行其一:" >&2
      echo "  sudo apt install -y ros-humble-sophus" >&2
      echo "  bash scripts/patch_kiss_icp_humble.sh   # 下载到 ${SOPHUS_CACHE}" >&2
      exit 1
    fi
  fi
fi

# 若已 source 过 install，叠加 livox 等前缀
if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  set +u
  source "${REPO_ROOT}/install/setup.bash"
  set -u
else
  dong_source_humble || exit 1
fi

# 必须删掉 _deps，否则会继续用上次失败的 github 下载缓存
rm -rf "${REPO_ROOT}/build/kiss_icp" "${REPO_ROOT}/install/kiss_icp"

cd "${REPO_ROOT}"
echo ">>> colcon build kiss_icp（优先系统 Sophus，请勿 Ctrl+C）..."
colcon build --packages-select kiss_icp \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DUSE_SYSTEM_SOPHUS="${USE_SYSTEM_SOPHUS}" \
    -DUSE_SYSTEM_EIGEN3=ON \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPython3_EXECUTABLE=/usr/bin/python3

echo "完成。请: source ${REPO_ROOT}/install/setup.bash"
