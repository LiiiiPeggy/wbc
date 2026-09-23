#!/usr/bin/env bash
# ################################
# Bash: laptop camera topic watch begin
# ################################
# Laptop only — robot must already run agilex_ws/scripts/run_camera.sh.
# ################################
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/_common.sh"
remani_require_ros

COLOR="${CAMERA_COLOR_TOPIC:-/camera/color/image_raw}"
DEPTH="${CAMERA_DEPTH_TOPIC:-/camera/depth/image_rect_raw}"

echo "[laptop] /camera topics:"
rostopic list 2>/dev/null | grep -E '^/camera' || echo "  (none)"

echo "[laptop] hz ${COLOR}"
rostopic info "${COLOR}" >/dev/null 2>&1 || {
  echo "ERROR: ${COLOR} not advertised — start robot run_camera.sh first" >&2
  exit 1
}
rostopic hz "${COLOR}" &
COLOR_PID=$!
trap 'kill ${COLOR_PID} 2>/dev/null || true' EXIT

if rostopic info "${DEPTH}" >/dev/null 2>&1; then
  echo "[laptop] hz ${DEPTH}"
  rostopic hz "${DEPTH}"
else
  wait "${COLOR_PID}"
fi
# ################################
# Bash: laptop camera topic watch end
# ################################
