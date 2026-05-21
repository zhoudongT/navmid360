#!/usr/bin/env bash
# 释放 Go2 导航栈常用端口（9090 rosbridge / 8081 网页等）
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PID_SCRIPT="${ROOT}/scripts/pid_on_port.sh"

PORTS=("$@")
if [[ ${#PORTS[@]} -eq 0 ]]; then
  PORTS=(9090 9091 8888 8080 8081)
fi

_kill_pid() {
  local pid="$1"
  local sig="$2"
  [[ -n "${pid}" && "${pid}" =~ ^[0-9]+$ ]] || return 0
  kill "-${sig}" "${pid}" 2>/dev/null || true
}

_kill_by_port_proc() {
  local port="$1"
  [[ -x "${PID_SCRIPT}" ]] || return 0
  local pids
  pids="$("${PID_SCRIPT}" "${port}" 2>/dev/null || true)"
  for pid in ${pids}; do
    if [[ -r "/proc/${pid}/cmdline" ]]; then
      local cmd
      cmd="$(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null || true)"
      echo "  [free_nav_ports] 端口 ${port} <- pid ${pid}: ${cmd}"
    fi
    _kill_pid "${pid}" TERM
  done
  sleep 0.4
  pids="$("${PID_SCRIPT}" "${port}" 2>/dev/null || true)"
  for pid in ${pids}; do
    _kill_pid "${pid}" KILL
  done
}

_kill_port_fuser() {
  local port="$1"
  if command -v fuser >/dev/null 2>&1; then
    fuser -k "${port}"/tcp 2>/dev/null || true
  fi
  if command -v ss >/dev/null 2>&1; then
    local pids
    pids=$(ss -tlnp 2>/dev/null | grep ":${port} " | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u)
    for pid in ${pids}; do
      _kill_pid "${pid}" TERM
    done
    sleep 0.3
    pids=$(ss -tlnp 2>/dev/null | grep ":${port} " | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u)
    for pid in ${pids}; do
      _kill_pid "${pid}" KILL
    done
  fi
}

pkill -TERM -f "rosbridge_websocket" 2>/dev/null || true
pkill -TERM -f "rosapi_node" 2>/dev/null || true
pkill -TERM -f "run_rosbridge_safe" 2>/dev/null || true
sleep 0.5
pkill -KILL -f "rosbridge_websocket" 2>/dev/null || true
pkill -KILL -f "rosapi_node" 2>/dev/null || true
pkill -KILL -f "run_rosbridge_safe" 2>/dev/null || true

for port in "${PORTS[@]}"; do
  _kill_by_port_proc "${port}"
  _kill_port_fuser "${port}"
done

sleep 0.5
