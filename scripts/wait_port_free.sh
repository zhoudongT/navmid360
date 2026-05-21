#!/usr/bin/env bash
# 等待 TCP 端口空闲（启动 rosbridge 前调用）
set -euo pipefail

PORT="${1:?用法: wait_port_free.sh <port> [max_sec]}"
MAX_SEC="${2:-25}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

_port_busy() {
  ss -tln 2>/dev/null | grep -q ":${PORT} "
}

if [[ -x "${ROOT}/scripts/free_nav_ports.sh" ]]; then
  "${ROOT}/scripts/free_nav_ports.sh" "${PORT}"
fi

for ((i = 0; i < MAX_SEC; i++)); do
  if ! _port_busy; then
    exit 0
  fi
  if (( i % 4 == 0 )); then
    "${ROOT}/scripts/free_nav_ports.sh" "${PORT}" 2>/dev/null || true
  fi
  sleep 1
done

echo "错误: 端口 ${PORT} 在 ${MAX_SEC}s 内仍被占用:" >&2
ss -tlnp 2>/dev/null | grep ":${PORT} " >&2 || true
if [[ -x "${ROOT}/scripts/pid_on_port.sh" ]]; then
  echo "占用进程 (pid_on_port):" >&2
  for pid in $("${ROOT}/scripts/pid_on_port.sh" "${PORT}" 2>/dev/null || true); do
    echo "  pid ${pid}: $(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null || echo '?')" >&2
  done
fi
echo "可尝试: sudo lsof -i :${PORT}  或  ROSBRIDGE_PORT=9091 ./scripts/start_go2_nav.sh" >&2
exit 1
