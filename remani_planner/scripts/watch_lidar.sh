#!/usr/bin/env bash
# ################################
# Bash: laptop lidar topic watch begin
# ################################
# Laptop only — robot must already run agilex_ws/scripts/run_lidar.sh.
# ################################
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/_common.sh"
remani_require_ros

TOPIC="${LIDAR_TOPIC:-/rslidar_points}"
echo "[laptop] hz ${TOPIC}"
rostopic info "${TOPIC}" 2>/dev/null || {
  echo "ERROR: ${TOPIC} not advertised — start robot run_lidar.sh first" >&2
  exit 1
}
rostopic hz "${TOPIC}"
# ################################
# Bash: laptop lidar topic watch end
# ################################
