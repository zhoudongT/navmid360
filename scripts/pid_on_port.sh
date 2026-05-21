#!/usr/bin/env bash
# 查出监听 TCP 端口的 PID（不依赖 ss -p / sudo）
# 用法: ./scripts/pid_on_port.sh 9090
set -euo pipefail

PORT="${1:?用法: pid_on_port.sh <port>}"
HEX="$(python3 -c "import socket; print('%04X' % socket.htons(int('${PORT}')))")"

declare -A INODES=()
while read -r line; do
  [[ "$line" =~ ^[[:space:]]*sl ]] && continue
  local_addr="$(echo "$line" | awk '{print $2}')"
  inode="$(echo "$line" | awk '{print $10}')"
  [[ "${local_addr##*:}" == "${HEX}" ]] || continue
  state="$(echo "$line" | awk '{print $4}')"
  [[ "$state" == "0A" ]] || continue
  INODES["$inode"]=1
done < /proc/net/tcp

if [[ ${#INODES[@]} -eq 0 ]]; then
  exit 1
fi

for pid_dir in /proc/[0-9]*; do
  pid="${pid_dir##*/}"
  [[ -d "${pid_dir}/fd" ]] || continue
  for fd in "${pid_dir}"/fd/*; do
    link="$(readlink "$fd" 2>/dev/null || true)"
    for inode in "${!INODES[@]}"; do
      if [[ "$link" == "socket:[${inode}]" ]]; then
        echo "$pid"
      fi
    done
  done
done | sort -u
