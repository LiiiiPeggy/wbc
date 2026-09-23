#!/usr/bin/env bash
# ################################
# Bash: robot-host camera topic watch begin
# ################################
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/_common.sh"
remani_require_ros

COLOR="${CAMERA_COLOR_TOPIC:-/camera/color/image_raw}"
DEPTH="${CAMERA_DEPTH_TOPIC:-/camera/depth/image_rect_raw}"

echo "[watch_camera] listing /camera topics:"
rostopic list 2>/dev/null | grep -E '^/camera' || echo "  (none yet)"

echo "[watch_camera] hz ${COLOR}"
if ! rostopic info "${COLOR}" >/dev/null 2>&1; then
  echo "ERROR: ${COLOR} not advertised — run run_camera.sh first" >&2
  exit 1
fi
rostopic hz "${COLOR}" &
COLOR_PID=$!
trap 'kill ${COLOR_PID} 2>/dev/null || true' EXIT

if rostopic info "${DEPTH}" >/dev/null 2>&1; then
  echo "[watch_camera] hz ${DEPTH}"
  rostopic hz "${DEPTH}"
else
  echo "[watch_camera] ${DEPTH} absent; holding on color hz only"
  wait "${COLOR_PID}"
fi
# ################################
# Bash: robot-host camera topic watch end
# ################################
