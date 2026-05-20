#!/usr/bin/env bash
# 克隆 [PolarisXQ/Fast-LIO2-Localization](https://github.com/PolarisXQ/Fast-LIO2-Localization) 到本仓库根目录下的 Fast-LIO2-Localization/，
# 内含修改版 FAST_LIO（包名仍为 fast_lio）与 icp_relocalization。
#
# 重要：与 Ericsii/FAST_LIO_ROS2 的 fast_lio 不能在同一工作区同时编译，请移除或移走其一后再 colcon build。
#
# 用法:
#   cd /path/to/dong_3d_nav
#   bash scripts/sync_fastlio2_localization.sh
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${1:-${REPO_ROOT}}"
LOC_DIR="${DEST}/Fast-LIO2-Localization"
URL="https://github.com/PolarisXQ/Fast-LIO2-Localization.git"
BRANCH="master"
MAX_RETRIES="${GIT_CLONE_RETRIES:-5}"

# 大仓库 / 不稳定网络：减轻 TLS 断连（curl 56 / index-pack 失败）
export GIT_HTTP_LOW_SPEED_LIMIT="${GIT_HTTP_LOW_SPEED_LIMIT:-1000}"
export GIT_HTTP_LOW_SPEED_TIME="${GIT_HTTP_LOW_SPEED_TIME:-600}"

_git_clone_retry() {
  local url="$1" branch="$2" dir="$3"
  local attempt
  for attempt in $(seq 1 "${MAX_RETRIES}"); do
    echo "[sync-fl2loc] 尝试 ${attempt}/${MAX_RETRIES}: git clone -> ${dir}"
    if git -c http.postBuffer=524288000 \
        -c http.version=HTTP/1.1 \
        clone --depth 1 --single-branch -b "${branch}" "${url}" "${dir}"; then
      return 0
    fi
    echo "[sync-fl2loc] 克隆失败，清理不完整目录后重试…" >&2
    rm -rf "${dir}"
    sleep $((attempt * 3))
  done
  return 1
}

_download_zip_fallback() {
  local dir="$1"
  local zip_url="https://github.com/PolarisXQ/Fast-LIO2-Localization/archive/refs/heads/${BRANCH}.zip"
  local tmp_zip
  tmp_zip="$(mktemp /tmp/Fast-LIO2-Localization-XXXXXX.zip)"
  local parent extracted
  parent="$(dirname "${dir}")"
  extracted="${parent}/Fast-LIO2-Localization-${BRANCH}"

  echo "[sync-fl2loc] git clone 失败，改用 GitHub ZIP 下载（无 .git，可正常 colcon build）…"
  rm -rf "${dir}" "${extracted}"
  if ! curl -fsSL --retry 5 --retry-delay 5 -o "${tmp_zip}" "${zip_url}"; then
    echo "[sync-fl2loc] ZIP 下载失败: ${zip_url}" >&2
    rm -f "${tmp_zip}"
    return 1
  fi
  if ! unzip -q -o "${tmp_zip}" -d "${parent}"; then
    echo "[sync-fl2loc] 解压失败: ${tmp_zip}" >&2
    rm -f "${tmp_zip}"
    return 1
  fi
  rm -f "${tmp_zip}"
  if [[ ! -d "${extracted}" ]]; then
    echo "[sync-fl2loc] 解压后未找到 ${extracted}" >&2
    return 1
  fi
  mv "${extracted}" "${dir}"
  return 0
}

if [[ -d "${LOC_DIR}/.git" ]]; then
  echo "[sync-fl2loc] 已存在 ${LOC_DIR}，跳过克隆（更新请在该目录 git pull）。"
elif [[ -d "${LOC_DIR}" ]]; then
  echo "[sync-fl2loc] 发现不完整目录（无 .git），删除后重新克隆: ${LOC_DIR}"
  rm -rf "${LOC_DIR}"
  if ! _git_clone_retry "${URL}" "${BRANCH}" "${LOC_DIR}"; then
    _download_zip_fallback "${LOC_DIR}" || exit 1
  fi
else
  if ! _git_clone_retry "${URL}" "${BRANCH}" "${LOC_DIR}"; then
    _download_zip_fallback "${LOC_DIR}" || exit 1
  fi
fi

ICP_DIR="${LOC_DIR}/icp_relocalization"
if [[ -f "${ICP_DIR}/CMakeLists.txt" ]]; then
  bash "${REPO_ROOT}/scripts/patch_icp_relocalization_humble.sh" "${ICP_DIR}"
fi

FAST_LIO_CMAKE="${LOC_DIR}/FAST_LIO/CMakeLists.txt"
if [[ -f "${FAST_LIO_CMAKE}" ]]; then
  echo "[sync-fl2loc] 对 FAST_LIO 应用 Humble + rosidl 补丁…"
  bash "${REPO_ROOT}/scripts/patch_fast_lio_polaris_humble.sh" "${LOC_DIR}/FAST_LIO"
  python3 "${REPO_ROOT}/scripts/patch_fast_lio_ros2_cmake.py" "${LOC_DIR}/FAST_LIO"
fi

echo
echo "[sync-fl2loc] 完成。编译示例（勿与 Ericsii fast_lio 同时放入 src）："
echo "  source /opt/ros/\${ROS_DISTRO}/setup.bash"
echo "  bash scripts/build_icp_relocalization_humble.sh"
echo "  bash scripts/build_fast_lio_polaris_humble.sh"
echo
echo "请先编辑 Fast-LIO2-Localization 内地图路径，见仓库 README 与 example.launch.py；Mid-360 需先起 Livox："
echo "  ros2 launch dong_octomap mid360_fastlio2_localization.launch.py"
echo
