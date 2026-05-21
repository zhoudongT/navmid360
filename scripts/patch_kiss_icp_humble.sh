#!/usr/bin/env bash
# Ubuntu 22.04 / CMake 3.22：将 KISS-ICP 依赖 Sophus 降为 1.22.10（避免 1.24 要求 CMake 3.24+）
set -eo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOPHUS_CMAKE="${REPO_ROOT}/KISS-ICP/cpp/kiss_icp/3rdparty/sophus/sophus.cmake"
if [[ ! -f "${SOPHUS_CMAKE}" ]]; then
  echo "未找到 ${SOPHUS_CMAKE}，请先 bash scripts/sync_kiss_icp.sh" >&2
  exit 1
fi
if grep -q '1.22.10' "${SOPHUS_CMAKE}"; then
  echo "[patch-kiss-icp] Sophus 已为 1.22.10，跳过"
  exit 0
fi
sed -i 's|refs/tags/1.24.6.tar.gz|refs/tags/1.22.10.tar.gz|' "${SOPHUS_CMAKE}"
echo "[patch-kiss-icp] 已将 Sophus 固定为 1.22.10"
