#!/usr/bin/env bash
# 单实例 rosbridge：先释放端口再启动，避免 launch 内 [Errno 98] 重试刷屏
set -euo pipefail

PORT="${1:-9090}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "错误: 需要 ROS 2 Humble" >&2
  exit 1
fi
# shellcheck source=/dev/null
set +u
source /opt/ros/humble/setup.bash
if [[ -f "${ROOT}/install/setup.bash" ]]; then
  source "${ROOT}/install/setup.bash"
fi
set -u

if ! ros2 pkg prefix rosbridge_server &>/dev/null; then
  echo "错误: 未安装 rosbridge_server" >&2
  exit 1
fi

"${ROOT}/scripts/wait_port_free.sh" "${PORT}" 20

echo "[run_rosbridge_safe] 启动 rosbridge WebSocket 端口 ${PORT}"
exec ros2 run rosbridge_server rosbridge_websocket \
  --ros-args -p "port:=${PORT}"
