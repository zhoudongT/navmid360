#!/usr/bin/env bash
# 将 Fast-LIO2（Ericsii/FAST_LIO_ROS2 ros2 → 目录名 fast_lio）与
# Fast-LIO2-Localization（PolarisXQ → 目录名 Fast-LIO2-Localization/）同步克隆到本工程目录。
#
# 二者可同时存在于磁盘；但两个树里都带有 ROS 包名 fast_lio，colcon 同一工作区只能编译其一，
# 请用 COLCON_IGNORE 或移走未使用的那一套。
#
# 用法:
#   cd /path/to/dong_3d_nav
#   bash scripts/sync_fastlio2_and_localization.sh
#
# 或指定目标根目录（默认为本仓库根）:
#   bash scripts/sync_fastlio2_and_localization.sh /path/to/parent
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${1:-${REPO_ROOT}}"
mkdir -p "${DEST}"

MAX_RETRIES="${GIT_CLONE_RETRIES:-5}"
export GIT_HTTP_LOW_SPEED_LIMIT="${GIT_HTTP_LOW_SPEED_LIMIT:-1000}"
export GIT_HTTP_LOW_SPEED_TIME="${GIT_HTTP_LOW_SPEED_TIME:-600}"

clone_shallow() {
  local url="$1" branch="$2" dir="$3"
  local attempt
  if [[ -d "${dir}/.git" ]]; then
    echo "[sync-fl2] 已存在 ${dir}，跳过克隆。"
    return 0
  fi
  if [[ -d "${dir}" ]]; then
    echo "[sync-fl2] 发现不完整目录（无 .git），删除后重新克隆: ${dir}"
    rm -rf "${dir}"
  fi
  for attempt in $(seq 1 "${MAX_RETRIES}"); do
    echo "[sync-fl2] 尝试 ${attempt}/${MAX_RETRIES}: git clone ${url} (-b ${branch}) -> ${dir}"
    if git -c http.postBuffer=524288000 \
        -c http.version=HTTP/1.1 \
        clone --depth 1 --single-branch -b "${branch}" "${url}" "${dir}"; then
      return 0
    fi
    echo "[sync-fl2] 克隆失败，清理后重试…" >&2
    rm -rf "${dir}"
    sleep $((attempt * 3))
  done
  echo "[sync-fl2] 错误: ${dir} 克隆失败。" >&2
  return 1
}

# --- Fast-LIO2（Ericsii，与 Mid360 常用 livox + mid360.yaml 配套）---
FAST_LIO_URL="https://github.com/Ericsii/FAST_LIO_ROS2.git"
FAST_LIO_BRANCH="ros2"
FAST_LIO_DIR="${DEST}/fast_lio"
clone_shallow "${FAST_LIO_URL}" "${FAST_LIO_BRANCH}" "${FAST_LIO_DIR}"
if [[ -f "${FAST_LIO_DIR}/CMakeLists.txt" ]]; then
  echo "[sync-fl2] 对 ${FAST_LIO_DIR}/CMakeLists.txt 应用 ROS2 CMake 补丁…"
  python3 "${REPO_ROOT}/scripts/patch_fast_lio_ros2_cmake.py" "${FAST_LIO_DIR}" || {
    echo "[sync-fl2] 警告: fast_lio CMake 补丁失败，请手动: python3 scripts/patch_fast_lio_ros2_cmake.py ${FAST_LIO_DIR}" >&2
  }
fi

# --- Fast-LIO2-Localization（Polaris，内含 FAST_LIO + icp_relocalization）---
bash "${REPO_ROOT}/scripts/sync_fastlio2_localization.sh" "${DEST}"

echo
echo "[sync-fl2] 全部完成。目录:"
echo "  - ${FAST_LIO_DIR}          (Ericsii Fast-LIO2 / ROS2)"
echo "  - ${DEST}/Fast-LIO2-Localization  (Polaris Fast-LIO2-Localization)"
echo "编译时请二选一 fast_lio，或使用 --paths 只指向其中一棵。详见 README。"
