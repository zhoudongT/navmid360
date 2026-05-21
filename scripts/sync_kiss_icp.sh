#!/usr/bin/env bash
# 克隆 PRBonn/kiss-icp 到本仓库 KISS-ICP/（ROS 2 包位于 KISS-ICP/ros，包名 kiss_icp）
#
# 用法:
#   cd /path/to/navmid360
#   bash scripts/sync_kiss_icp.sh
#
set -euo pipefail

# 避免失效的 git/curl 代理（如 127.0.0.1:7897）导致克隆失败
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy || true

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${1:-${REPO_ROOT}}"
KISS_DIR="${DEST}/KISS-ICP"
URL="https://github.com/PRBonn/kiss-icp.git"
BRANCH="main"
MAX_RETRIES="${GIT_CLONE_RETRIES:-5}"

export GIT_HTTP_LOW_SPEED_LIMIT="${GIT_HTTP_LOW_SPEED_LIMIT:-1000}"
export GIT_HTTP_LOW_SPEED_TIME="${GIT_HTTP_LOW_SPEED_TIME:-600}"

_git_clone_retry() {
  local url="$1" branch="$2" dir="$3"
  local attempt
  for attempt in $(seq 1 "${MAX_RETRIES}"); do
    echo "[sync-kiss-icp] 尝试 ${attempt}/${MAX_RETRIES}: git clone -> ${dir}"
    if git -c http.postBuffer=524288000 \
        -c http.version=HTTP/1.1 \
        clone --depth 1 --single-branch -b "${branch}" "${url}" "${dir}"; then
      return 0
    fi
    echo "[sync-kiss-icp] 克隆失败，清理后重试…" >&2
    rm -rf "${dir}"
    sleep $((attempt * 3))
  done
  return 1
}

_download_zip_fallback() {
  local dir="$1"
  local zip_url="https://github.com/PRBonn/kiss-icp/archive/refs/heads/${BRANCH}.zip"
  local tmp_zip parent extracted
  tmp_zip="$(mktemp /tmp/kiss-icp-XXXXXX.zip)"
  parent="$(dirname "${dir}")"
  extracted="${parent}/kiss-icp-${BRANCH}"

  echo "[sync-kiss-icp] git clone 失败，改用 GitHub ZIP…"
  rm -rf "${dir}" "${extracted}"
  curl -fsSL --retry 5 --retry-delay 5 -o "${tmp_zip}" "${zip_url}"
  unzip -q -o "${tmp_zip}" -d "${parent}"
  rm -f "${tmp_zip}"
  mv "${extracted}" "${dir}"
}

if [[ -d "${KISS_DIR}/.git" ]]; then
  echo "[sync-kiss-icp] 已存在 ${KISS_DIR}，跳过克隆（更新: cd ${KISS_DIR} && git pull）"
elif [[ -d "${KISS_DIR}" ]]; then
  rm -rf "${KISS_DIR}"
  _git_clone_retry "${URL}" "${BRANCH}" "${KISS_DIR}" || _download_zip_fallback "${KISS_DIR}"
else
  _git_clone_retry "${URL}" "${BRANCH}" "${KISS_DIR}" || _download_zip_fallback "${KISS_DIR}"
fi

if [[ ! -f "${KISS_DIR}/ros/package.xml" ]]; then
  echo "[sync-kiss-icp] 错误: 未找到 ${KISS_DIR}/ros/package.xml" >&2
  exit 1
fi

bash "${REPO_ROOT}/scripts/patch_kiss_icp_humble.sh"

echo
echo "[sync-kiss-icp] 完成。编译:"
echo "  source /opt/ros/\${ROS_DISTRO}/setup.bash"
echo "  bash scripts/build_kiss_icp_humble.sh"
echo
echo "一键定位（与 fastlio2 相同流程: Livox + ICP + KISS-ICP）:"
echo "  ros2 launch dong_octomap mid360_kiss_icp_localization.launch.py"
