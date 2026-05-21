#!/usr/bin/env bash
# 启动 fast_lio relocalization.launch.py，并规避海康 MVS 旧 libusb 导致 PCL 崩溃。
#
# 用法:
#   bash scripts/run_fastlio_relocalization.sh prior_pcd_path:=/home/allen/maps/scans_0.5m.pcd
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "${REPO_ROOT}/scripts/source_dong_humble.sh"

exec ros2 launch fast_lio relocalization.launch.py "$@"
