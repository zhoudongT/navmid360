#!/usr/bin/env bash
# 停止 Go2 / Fast-LIO2 导航相关进程并释放 8888(网页)/9090(ws) 等端口
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${ROOT}/scripts/go2_nav_kill.sh" "$@"
