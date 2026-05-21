#!/usr/bin/env bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KISS_DIR="${REPO_ROOT}/KISS-ICP"

if [[ ! -f "${KISS_DIR}/ros/package.xml" ]]; then
  echo "未找到 KISS-ICP/ros，请先: bash scripts/sync_kiss_icp.sh" >&2
  exit 1
fi

bash "${REPO_ROOT}/scripts/patch_kiss_icp_humble.sh"
rm -rf "${REPO_ROOT}/build/kiss_icp" "${REPO_ROOT}/install/kiss_icp"

source /opt/ros/humble/setup.bash
cd "${REPO_ROOT}"
colcon build --packages-select kiss_icp --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "完成。请: source ${REPO_ROOT}/install/setup.bash"
