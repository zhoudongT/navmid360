#!/usr/bin/env bash
# 同步 Go2 + CHAMP Gazebo 栈（go2_nav2_ros2），并生成带 Mid-360 的 URDF。
#
# 来源: https://github.com/arjun-sadananda/go2_nav2_ros2 (branch: main)
#
# 用法:
#   bash scripts/sync_go2_champ_gazebo.sh              # 默认 --fast（压缩包）
#   bash scripts/sync_go2_champ_gazebo.sh --build
#   bash scripts/sync_go2_champ_gazebo.sh --git
#   bash scripts/sync_go2_champ_gazebo.sh --force
#
# 环境变量:
#   GO2_CHAMP_MIRROR=https://ghfast.top/
#   GO2_CHAMP_REPO=/path/to/go2_nav2_ros2   # 本地克隆，跳过下载
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${REPO_ROOT}/third_party/go2_champ_sim"
OWNER_REPO="arjun-sadananda/go2_nav2_ros2"
BRANCH="main"
CACHE="${REPO_ROOT}/.cache"
TMP="${CACHE}/go2_nav2_ros2"
ARCHIVE="${CACHE}/go2_nav2_ros2-${BRANCH}.tar.gz"
EXTRACT_DIR="${CACHE}/go2_nav2_ros2-${BRANCH}"
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

  --fast   默认：下载 main 分支 tar.gz
  --git    git sparse-checkout（champ / go2_description / aws 仓库）
  --force  忽略本地缓存重新下载
  --build  同步后执行 build_go2_champ_gazebo.sh

环境变量:
  GO2_CHAMP_MIRROR      镜像前缀，如 https://ghfast.top/
  GO2_CHAMP_ARCHIVE_URL 完整 tar.gz URL
  GO2_CHAMP_REPO        本地已有 go2_nav2_ros2 路径
  GO2_CHAMP_NO_PROXY=1  禁用 http(s)_proxy

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
      -o "$(basename "${out}")" -d "$(dirname "${out}")" "${url}" >&2
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
    if [[ ! -d "${EXTRACT_DIR}" ]]; then
      local found
      found="$(find "${CACHE}" -maxdepth 1 -type d -name 'go2_nav2_ros2-*' | head -1)"
      if [[ -n "${found}" ]]; then
        mv "${found}" "${EXTRACT_DIR}"
      else
        echo "[sync-go2-champ] 错误: 解压后未找到 ${EXTRACT_DIR}" >&2
        exit 1
      fi
    fi
  fi
  echo "${EXTRACT_DIR}" >&1
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
      champ \
      go2_description \
      aws-robomaker-small-warehouse-world
    git -C "${TMP}" checkout "${BRANCH}"
  else
    echo "[sync-go2-champ] 更新 git 缓存 ${TMP}…" >&2
    git -C "${TMP}" fetch origin "${BRANCH}" --depth 1 2>/dev/null || true
    git -C "${TMP}" checkout "${BRANCH}" 2>/dev/null || true
  fi
  echo "${TMP}" >&1
}

_ensure_src() {
  if [[ -n "${SRC_REPO}" && -d "${SRC_REPO}/champ" ]]; then
    echo "[sync-go2-champ] 使用本地仓库 ${SRC_REPO}" >&2
    echo "${SRC_REPO}" >&1
    return 0
  fi
  if [[ "${USE_FAST}" == "true" ]]; then
    _ensure_from_archive
    return 0
  fi
  _ensure_from_git_sparse
}

ROOT="$(_ensure_src)"
SRC_DESC="${ROOT}/go2_description"
SRC_CHAMP="${ROOT}/champ"
SRC_AWS="${ROOT}/aws-robomaker-small-warehouse-world"
if [[ ! -d "${SRC_DESC}/xacro" || ! -d "${SRC_CHAMP}/champ_config" ]]; then
  echo "[sync-go2-champ] 错误: 缺少 go2_description 或 champ/champ_config（ROOT=${ROOT}）" >&2
  exit 1
fi

mkdir -p "${DEST}"
echo "[sync-go2-champ] rsync champ -> ${DEST}/champ …"
rsync -a --delete "${SRC_CHAMP}/" "${DEST}/champ/"
if [[ -d "${SRC_AWS}" ]]; then
  echo "[sync-go2-champ] rsync aws warehouse -> ${DEST}/aws_robomaker_small_warehouse_world …"
  rsync -a --delete "${SRC_AWS}/" "${DEST}/aws_robomaker_small_warehouse_world/"
fi
# 移除旧 anujjain 栈残留
rm -rf "${DEST}/go2_config" "${DEST}/champ_teleop"
date -Iseconds > "${DEST}/.synced_at"

echo "[sync-go2-champ] 更新 go2_description …"
mkdir -p "${GO2_DESC}"
_NAVMID360_PKG_XML="${CACHE}/go2_description.package.xml.bak"
_NAVMID360_CMAKE="${CACHE}/go2_description.CMakeLists.txt.bak"
_ROS_CTRL_SIM="${GO2_DESC}/config/ros_control/ros_control_sim.yaml"
_MID360_MOUNT="${GO2_DESC}/xacro/mid360_head_mount.xacro"
_MID360_MESH="${GO2_DESC}/meshes/mid360.stl"
[[ -f "${GO2_DESC}/package.xml" ]] && cp -f "${GO2_DESC}/package.xml" "${_NAVMID360_PKG_XML}" || true
[[ -f "${GO2_DESC}/CMakeLists.txt" ]] && cp -f "${GO2_DESC}/CMakeLists.txt" "${_NAVMID360_CMAKE}" || true
[[ -f "${_ROS_CTRL_SIM}" ]] && cp -f "${_ROS_CTRL_SIM}" "${CACHE}/ros_control_sim.yaml.bak" || true
[[ -f "${_MID360_MOUNT}" ]] && cp -f "${_MID360_MOUNT}" "${CACHE}/mid360_head_mount.xacro.bak" || true
[[ -f "${_MID360_MESH}" ]] && cp -f "${_MID360_MESH}" "${CACHE}/mid360.stl.bak" || true

rsync -a --delete --exclude '.git' "${SRC_DESC}/" "${GO2_DESC}/"

if [[ ! -f "${GO2_DESC}/package.xml" && -f "${_NAVMID360_PKG_XML}" ]]; then
  cp -f "${_NAVMID360_PKG_XML}" "${GO2_DESC}/package.xml"
fi
if [[ -f "${GO2_DESC}/CMakeLists.txt" ]] && ! grep -qE 'install\(DIRECTORY.*(meshes|xacro)' "${GO2_DESC}/CMakeLists.txt" 2>/dev/null; then
  if [[ -f "${_NAVMID360_CMAKE}" ]] && grep -qE 'install\(DIRECTORY.*(meshes|xacro)' "${_NAVMID360_CMAKE}" 2>/dev/null; then
    cp -f "${_NAVMID360_CMAKE}" "${GO2_DESC}/CMakeLists.txt"
  fi
fi
[[ -f "${CACHE}/ros_control_sim.yaml.bak" ]] && mkdir -p "${GO2_DESC}/config/ros_control" && cp -f "${CACHE}/ros_control_sim.yaml.bak" "${_ROS_CTRL_SIM}" || true
[[ -f "${CACHE}/mid360_head_mount.xacro.bak" ]] && cp -f "${CACHE}/mid360_head_mount.xacro.bak" "${_MID360_MOUNT}" || true
[[ -f "${CACHE}/mid360.stl.bak" ]] && cp -f "${CACHE}/mid360.stl.bak" "${_MID360_MESH}" || true

python3 "${REPO_ROOT}/scripts/patch_go2_robot_mid360_xacro.py" "${GO2_DESC}"

echo
echo "[sync-go2-champ] 完成（来源: ${OWNER_REPO} @ ${BRANCH}）"
echo "  CHAMP 栈: ${DEST}/champ"
echo "  URDF 包:  ${GO2_DESC}"
echo "  Mid-360:  ${GO2_DESC}/xacro/robot_mid360.xacro"
echo
echo "系统依赖:"
echo "  sudo apt install -y ros-humble-gazebo-ros2-control ros-humble-ros2-control \\"
echo "    ros-humble-ros2-controllers ros-humble-xacro ros-humble-robot-localization \\"
echo "    ros-humble-topic-tools"
echo
echo "编译: bash ${REPO_ROOT}/scripts/build_go2_champ_gazebo.sh"
echo
echo "楼梯仿真:"
echo "  ./scripts/run_legged_sim_2_storey.sh --sim-only"
echo

if [[ "${DO_BUILD}" == "true" ]]; then
  bash "${REPO_ROOT}/scripts/build_go2_champ_gazebo.sh"
fi
