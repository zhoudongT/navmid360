#!/usr/bin/env bash
# Ubuntu 22.04 / Humble：
# - 优先使用 apt 的 ros-humble-sophus（避免 CMake 在线拉 GitHub）
# - 若需源码构建：预下载 Sophus 1.22.10 到 KISS-ICP/.cache/，改 FetchContent 为本地 file://
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOPHUS_CMAKE="${REPO_ROOT}/KISS-ICP/cpp/kiss_icp/3rdparty/sophus/sophus.cmake"
CACHE_DIR="${REPO_ROOT}/KISS-ICP/.cache"
SOPHUS_TAR="${CACHE_DIR}/sophus-1.22.10.tar.gz"
GITHUB_URL="https://github.com/strasdat/Sophus/archive/refs/tags/1.22.10.tar.gz"

if [[ ! -f "${SOPHUS_CMAKE}" ]]; then
  echo "未找到 ${SOPHUS_CMAKE}，请先: bash scripts/sync_kiss_icp.sh" >&2
  exit 1
fi

unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy || true

download_sophus_tarball() {
  if [[ -f "${SOPHUS_TAR}" ]] && [[ "$(stat -c%s "${SOPHUS_TAR}" 2>/dev/null || echo 0)" -gt 10000 ]]; then
    echo "[patch-kiss-icp] 已存在 Sophus 缓存: ${SOPHUS_TAR}"
    return 0
  fi
  mkdir -p "${CACHE_DIR}"
  local url
  for url in \
    "https://ghfast.top/${GITHUB_URL}" \
    "${GITHUB_URL}"; do
    echo "[patch-kiss-icp] 下载 Sophus: ${url}"
    if curl -fSL --http1.1 --retry 3 --connect-timeout 30 --max-time 600 \
        -o "${SOPHUS_TAR}.part" "${url}" \
        && [[ -s "${SOPHUS_TAR}.part" ]]; then
      mv -f "${SOPHUS_TAR}.part" "${SOPHUS_TAR}"
      echo "[patch-kiss-icp] Sophus 缓存完成 ($(stat -c%s "${SOPHUS_TAR}") bytes)"
      return 0
    fi
    rm -f "${SOPHUS_TAR}.part"
    echo "[patch-kiss-icp] 下载失败，尝试下一镜像…" >&2
  done
  echo "[patch-kiss-icp] 无法下载 Sophus。可安装系统包后跳过下载:" >&2
  echo "  sudo apt install -y ros-humble-sophus" >&2
  return 1
}

# 将 sophus.cmake 改为：有本地缓存则用 file://，否则仍尝试镜像（构建时可能失败）
cat > "${SOPHUS_CMAKE}" <<'CMAKE_EOF'
# MIT License — patched by dong_3d_nav/scripts/patch_kiss_icp_humble.sh
include(FetchContent)

set(SOPHUS_USE_BASIC_LOGGING ON CACHE BOOL "Don't use fmt for Sophus libraru")
set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "Don't build Sophus tests")
set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "Don't build Sophus Examples")

get_filename_component(_DONG_SOPHUS_CACHE "${CMAKE_CURRENT_LIST_DIR}/../../../../.cache/sophus-1.22.10.tar.gz" ABSOLUTE)
if(EXISTS "${_DONG_SOPHUS_CACHE}")
  message(STATUS "dong_3d_nav: Using cached Sophus tarball ${_DONG_SOPHUS_CACHE}")
  FetchContent_Declare(sophus SYSTEM URL "file://${_DONG_SOPHUS_CACHE}")
else()
  message(STATUS "dong_3d_nav: Downloading Sophus via mirror (run patch_kiss_icp_humble.sh if this fails)")
  FetchContent_Declare(sophus SYSTEM
    URL https://ghfast.top/https://github.com/strasdat/Sophus/archive/refs/tags/1.22.10.tar.gz)
endif()
FetchContent_MakeAvailable(sophus)
CMAKE_EOF

echo "[patch-kiss-icp] 已更新 sophus.cmake（本地缓存 + 镜像回退）"

# 系统 Sophus 未安装时预下载，供 USE_SYSTEM_SOPHUS=OFF 时 FetchContent 使用
if ! dpkg -s ros-humble-sophus &>/dev/null; then
  download_sophus_tarball || true
else
  echo "[patch-kiss-icp] 已安装 ros-humble-sophus，编译将优先 USE_SYSTEM_SOPHUS=ON"
fi
