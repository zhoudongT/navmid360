#!/usr/bin/env bash
# 同步 Unitree Go2 + CHAMP 腿控 Gazebo 栈，并生成带 Mid-360 的 URDF。
#
# 来源: https://github.com/anujjain-dev/unitree-go2-ros2 (branch: humble)
#
# 用法:
#   bash scripts/sync_go2_champ_gazebo.sh              # 默认 --fast（压缩包，通常最快）
#   bash scripts/sync_go2_champ_gazebo.sh --build
#   bash scripts/sync_go2_champ_gazebo.sh --git         # 强制 git sparse（仅小目录）
#   bash scripts/sync_go2_champ_gazebo.sh --force       # 忽略缓存重新下载
#
# 加速（国内建议）:
#   export GO2_CHAMP_MIRROR=https://ghfast.top/
#   # 或: https://mirror.ghproxy.com/
#   bash scripts/sync_go2_champ_gazebo.sh --build
#
# 本地已有克隆可跳过下载:
#   export GO2_CHAMP_REPO=/path/to/unitree-go2-ros2
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${REPO_ROOT}/third_party/go2_champ_sim"
OWNER_REPO="anujjain-dev/unitree-go2-ros2"
BRANCH="humble"
CACHE="${REPO_ROOT}/.cache"
TMP="${CACHE}/unitree-go2-ros2"
ARCHIVE="${CACHE}/unitree-go2-ros2-${BRANCH}.tar.gz"
EXTRACT_DIR="${CACHE}/unitree-go2-ros2-${BRANCH}"
GO2_DESC="${REPO_ROOT}/go2_description"
SRC_REPO="${GO2_CHAMP_REPO:-}"
MIRROR="${GO2_CHAMP_MIRROR:-}"
ARCHIVE_URL="${GO2_CHAMP_ARCHIVE_URL:-}"
DO_BUILD=false
USE_FAST=true
FORCE=false

for arg in "$@"; do
  case "${arg}" in
    --build) DO_BUILD=true ;;
    --fast) USE_FAST=true ;;
    --git) USE_FAST=false ;;
    --force) FORCE=true ;;
    -h|--help)
      cat <<'EOF'
用法: bash scripts/sync_go2_champ_gazebo.sh [--build] [--fast|--git] [--force]

  --fast   默认：下载 humble 分支 tar.gz（比完整 git clone 快很多）
  --git    git sparse-checkout，只拉 champ / go2_config / go2_description
  --force  忽略本地缓存重新下载
  --build  同步后执行 build_go2_champ_gazebo.sh

环境变量:
  GO2_CHAMP_MIRROR      镜像前缀，如 https://ghfast.top/
  GO2_CHAMP_ARCHIVE_URL 完整 tar.gz URL（覆盖默认）
  GO2_CHAMP_REPO        本地已有仓库路径，跳过下载
  GO2_CHAMP_NO_PROXY=1  禁用 http(s)_proxy（默认保留代理以加速）

EOF
      exit 0
      ;;
    *)
      echo "未知参数: ${arg}" >&2
      exit 1
      ;;
  esac
done

if [[ "${GO2_CHAMP_NO_PROXY:-}" == "1" ]]; then
  unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy || true
fi

_default_archive_url() {
  local base="https://github.com/${OWNER_REPO}/archive/refs/heads/${BRANCH}.tar.gz"
  if [[ -n "${ARCHIVE_URL}" ]]; then
    echo "${ARCHIVE_URL}"
  elif [[ -n "${MIRROR}" ]]; then
    local m="${MIRROR}"
    [[ "${m}" != */ ]] && m="${m}/"
    echo "${m}${base}"
  else
    echo "${base}"
  fi
}

_download() {
  local url="$1" out="$2"
  mkdir -p "$(dirname "${out}")"
  echo "[sync-go2-champ] 下载 ${url}" >&2
  if command -v aria2c >/dev/null 2>&1; then
    aria2c -x16 -s16 -k1M --file-allocation=none -c \
      -o "$(basename "${out}")" -d "$(dirname "${out}")" "${url}"
    return 0
  fi
  if command -v curl >/dev/null 2>&1; then
    curl -fL --connect-timeout 20 --retry 3 --retry-delay 2 \
      -C - -o "${out}" "${url}"
    return 0
  fi
  if command -v wget >/dev/null 2>&1; then
    wget -c -O "${out}" "${url}"
    return 0
  fi
  echo "[sync-go2-champ] 错误: 需要 curl、wget 或 aria2c" >&2
  exit 1
}

_ensure_from_archive() {
  local url
  url="$(_default_archive_url)"
  if [[ "${FORCE}" == "true" ]]; then
    rm -f "${ARCHIVE}"
    rm -rf "${EXTRACT_DIR}"
  fi
  if [[ ! -f "${ARCHIVE}" ]]; then
    _download "${url}" "${ARCHIVE}"
  else
    echo "[sync-go2-champ] 使用缓存压缩包 ${ARCHIVE}" >&2
  fi
  if [[ ! -d "${EXTRACT_DIR}" ]]; then
    echo "[sync-go2-champ] 解压…" >&2
    rm -rf "${EXTRACT_DIR}"
    mkdir -p "${CACHE}"
    tar -xzf "${ARCHIVE}" -C "${CACHE}"
    # GitHub 解压目录名: unitree-go2-ros2-humble
    if [[ ! -d "${EXTRACT_DIR}" ]]; then
      local found
      found="$(find "${CACHE}" -maxdepth 1 -type d -name 'unitree-go2-ros2-*' | head -1)"
      if [[ -n "${found}" ]]; then
        mv "${found}" "${EXTRACT_DIR}"
      else
        echo "[sync-go2-champ] 错误: 解压后未找到 ${EXTRACT_DIR}" >&2
        exit 1
      fi
    fi
  fi
  echo "${EXTRACT_DIR}"
}

_ensure_from_git_sparse() {
  local url="https://github.com/${OWNER_REPO}.git"
  if [[ -n "${MIRROR}" ]]; then
    local m="${MIRROR}"
    [[ "${m}" != */ ]] && m="${m}/"
    url="${m}${url}"
  fi
  if [[ "${FORCE}" == "true" ]]; then
    rm -rf "${TMP}"
  fi
  if [[ ! -d "${TMP}/.git" ]]; then
    rm -rf "${TMP}"
    mkdir -p "$(dirname "${TMP}")"
    echo "[sync-go2-champ] git sparse clone ${url} (${BRANCH})…" >&2
    git -c http.postBuffer=524288000 -c http.version=HTTP/1.1 \
      clone --depth 1 --filter=blob:none --sparse -b "${BRANCH}" "${url}" "${TMP}"
    git -C "${TMP}" sparse-checkout set \
      champ champ_teleop \
      robots/configs/go2_config \
      robots/descriptions/go2_description
    git -C "${TMP}" checkout "${BRANCH}"
  else
    echo "[sync-go2-champ] 更新 git 缓存 ${TMP}…" >&2
    git -C "${TMP}" fetch origin "${BRANCH}" --depth 1 2>/dev/null || true
    git -C "${TMP}" checkout "${BRANCH}" 2>/dev/null || true
  fi
  echo "${TMP}"
}

_ensure_src() {
  if [[ -n "${SRC_REPO}" && -d "${SRC_REPO}/champ" ]]; then
    echo "[sync-go2-champ] 使用本地仓库 ${SRC_REPO}" >&2
    echo "${SRC_REPO}"
    return 0
  fi
  if [[ "${USE_FAST}" == "true" ]]; then
    _ensure_from_archive
    return 0
  fi
  _ensure_from_git_sparse
}

ROOT="$(_ensure_src)"
SRC_DESC="${ROOT}/robots/descriptions/go2_description"
SRC_CFG="${ROOT}/robots/configs/go2_config"
if [[ ! -d "${SRC_DESC}/xacro" || ! -d "${SRC_CFG}" ]]; then
  echo "[sync-go2-champ] 错误: 缺少 go2_description 或 go2_config（ROOT=${ROOT}）" >&2
  exit 1
fi

mkdir -p "${DEST}"
echo "[sync-go2-champ] rsync -> ${DEST} …"
rsync -a --delete "${ROOT}/champ/" "${DEST}/champ/"
rsync -a --delete "${ROOT}/champ_teleop/" "${DEST}/champ_teleop/"
rsync -a --delete "${SRC_CFG}/" "${DEST}/go2_config/"
date -Iseconds > "${DEST}/.synced_at"

echo "[sync-go2-champ] 更新 go2_description …"
mkdir -p "${GO2_DESC}"
_NAVMID360_PKG_XML="${CACHE}/go2_description.package.xml.bak"
_NAVMID360_CMAKE="${CACHE}/go2_description.CMakeLists.txt.bak"
[[ -f "${GO2_DESC}/package.xml" ]] && cp -f "${GO2_DESC}/package.xml" "${_NAVMID360_PKG_XML}" || true
[[ -f "${GO2_DESC}/CMakeLists.txt" ]] && cp -f "${GO2_DESC}/CMakeLists.txt" "${_NAVMID360_CMAKE}" || true
rsync -a --delete --exclude '.git' "${SRC_DESC}/" "${GO2_DESC}/"
if [[ ! -f "${GO2_DESC}/package.xml" && -f "${_NAVMID360_PKG_XML}" ]]; then
  cp -f "${_NAVMID360_PKG_XML}" "${GO2_DESC}/package.xml"
fi
if [[ -f "${GO2_DESC}/CMakeLists.txt" ]] && ! grep -qE 'install\(DIRECTORY.*(meshes|xacro)' "${GO2_DESC}/CMakeLists.txt" 2>/dev/null; then
  if [[ -f "${_NAVMID360_CMAKE}" ]] && grep -qE 'install\(DIRECTORY.*(meshes|xacro)' "${_NAVMID360_CMAKE}" 2>/dev/null; then
    cp -f "${_NAVMID360_CMAKE}" "${GO2_DESC}/CMakeLists.txt"
  fi
fi

python3 "${REPO_ROOT}/scripts/patch_go2_robot_mid360_xacro.py" "${GO2_DESC}"

echo
echo "[sync-go2-champ] 完成"
echo "  CHAMP 栈: ${DEST}"
echo "  URDF 包:  ${GO2_DESC}"
echo "  Mid-360:  ${GO2_DESC}/xacro/robot_mid360.xacro"
echo
echo "系统依赖:"
echo "  sudo apt install -y ros-humble-gazebo-ros2-control ros-humble-ros2-control \\"
echo "    ros-humble-ros2-controllers ros-humble-xacro ros-humble-robot-localization \\"
echo "    ros-humble-topic-tools"
echo "  # 可选更快下载: sudo apt install -y aria2"
echo
echo "编译: bash ${REPO_ROOT}/scripts/build_go2_champ_gazebo.sh"
echo
echo "楼梯仿真:"
echo "  ros2 launch dong_gazebo_sim mid360_sim_2_storey.launch.py robot_model:=legged legged_sim:=true \\"
echo "    icp_planar_constraint:=false stair_navigation:=true"
echo

if [[ "${DO_BUILD}" == "true" ]]; then
  bash "${REPO_ROOT}/scripts/build_go2_champ_gazebo.sh"
fi
