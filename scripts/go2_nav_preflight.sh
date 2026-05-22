#!/usr/bin/env bash
# 一键启动前检查：旧进程、端口、地图文件
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
WEB_PORT="${WEB_PORT:-8888}"
FAST_START="${FAST_START:-1}"
if [[ "${FAST_START}" == "1" ]]; then
  WEB_WAIT_SEC=2
  ROSBRIDGE_WAIT_SEC=3
else
  WEB_WAIT_SEC=8
  ROSBRIDGE_WAIT_SEC=12
fi
LOCK_FILE="/tmp/go2_nav_stack.lock"
PORT_FILE="/tmp/go2_nav_stack.rosbridge_port"

if [[ -f "${LOCK_FILE}" ]]; then
  old_pid="$(cat "${LOCK_FILE}" 2>/dev/null || true)"
  if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" 2>/dev/null; then
    echo "错误: 已有导航栈在运行 (pid=${old_pid})。请先:" >&2
    echo "  ./scripts/stop_go2_nav.sh" >&2
    exit 1
  fi
  rm -f "${LOCK_FILE}"
fi

# 仅在有旧 launch 时清理，避免误杀当前正在执行的 oneclick 脚本
if pgrep -f "ros2 launch go2_nav_stack" >/dev/null 2>&1; then
  if [[ -x "${ROOT}/scripts/go2_nav_kill.sh" ]]; then
    echo ">>> preflight: 发现旧导航 launch，正在结束..."
    "${ROOT}/scripts/go2_nav_kill.sh" || true
    sleep 2
  fi
else
  echo ">>> preflight: 无旧 ros2 launch，跳过全量 kill"
fi

"${ROOT}/scripts/free_nav_ports.sh" "${WEB_PORT}" "${ROSBRIDGE_PORT}" 9091 8888 8080 8081
if ! "${ROOT}/scripts/wait_port_free.sh" "${WEB_PORT}" "${WEB_WAIT_SEC}"; then
  echo "错误: 网页端口 ${WEB_PORT} 无法释放，请检查占用" >&2
  exit 1
fi
if ! "${ROOT}/scripts/wait_port_free.sh" "${ROSBRIDGE_PORT}" "${ROSBRIDGE_WAIT_SEC}"; then
  echo "警告: ${ROSBRIDGE_PORT} 仍被占用，改用 9091 启动 rosbridge（网页 ws 需填 ws://主机:9091）" >&2
  ROSBRIDGE_PORT=9091
  "${ROOT}/scripts/free_nav_ports.sh" 9091
  "${ROOT}/scripts/wait_port_free.sh" 9091 8 || true
fi

echo "${ROSBRIDGE_PORT}" >"${PORT_FILE}"
echo ">>> preflight: 端口 ${ROSBRIDGE_PORT} / ${WEB_PORT} 已就绪"
