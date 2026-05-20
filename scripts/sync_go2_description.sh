#!/usr/bin/env bash
# 克隆 Unitree Go2 URDF 包到本仓库，并可选 colcon 编译。
#
# 用法:
#   bash scripts/sync_go2_description.sh              # 仅克隆/更新
#   bash scripts/sync_go2_description.sh --build      # 克隆后 colcon build
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GO2_URL="https://github.com/Unitree-Go2-Robot/go2_description.git"
GO2_BRANCH="humble-devel"
GO2_DIR="${REPO_ROOT}/go2_description"
DO_BUILD=false

for arg in "$@"; do
  case "${arg}" in
    --build) DO_BUILD=true ;;
    -h|--help)
      echo "用法: bash scripts/sync_go2_description.sh [--build]"
      exit 0
      ;;
    *)
      echo "未知参数: ${arg}" >&2
      exit 1
      ;;
  esac
done

clone_or_update() {
  if [[ -d "${GO2_DIR}/.git" ]]; then
    echo "[sync] 已存在 ${GO2_DIR}，执行 git pull…"
    git -C "${GO2_DIR}" pull --ff-only
    return 0
  fi
  if [[ -e "${GO2_DIR}" ]]; then
    echo "[sync] 错误: ${GO2_DIR} 已存在且不是 git 仓库。" >&2
    exit 1
  fi
  echo "[sync] git clone ${GO2_URL} (-b ${GO2_BRANCH}) -> ${GO2_DIR}"
  git clone --depth 1 -b "${GO2_BRANCH}" "${GO2_URL}" "${GO2_DIR}"
}

clone_or_update

if [[ "${DO_BUILD}" == "true" ]]; then
  echo "[sync] colcon build --packages-select go2_description …"
  set +u
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  set -u
  cd "${REPO_ROOT}"
  colcon build --symlink-install --packages-select go2_description \
    --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
  echo "[sync] 编译完成。请执行: source ${REPO_ROOT}/install/setup.bash"
fi

echo
echo "[sync] go2_description 就绪: ${GO2_DIR}"
echo "  URDF: ${GO2_DIR}/urdf/go2_description.urdf"
echo "  若未使用 --build，请手动:"
echo "    source /opt/ros/humble/setup.bash && cd ${REPO_ROOT}"
echo "    colcon build --symlink-install --packages-select go2_description \\"
echo "      --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3"
echo "    source install/setup.bash"
