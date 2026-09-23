#!/usr/bin/env bash
# ################################
# Bash: laptop watch Ranger HW + dry-run preview begin
# ################################
# Under dry_run=true, HW topic should stay empty; preview may have twists.
#
# Usage:
#   remani_planner/scripts/watch_ranger_cmd.sh
# Env:
#   ALSO_ECHO_PREVIEW=0  set 0 to skip dry-run preview
# ################################
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/_common.sh"
remani_require_ros

HW_TOPIC="/remani/hardware/ranger/cmd_vel"
PREVIEW_TOPIC="/remani/dry_run/ranger_cmd_vel_preview"

echo "[laptop] HW topic info (${HW_TOPIC}) — should have no publisher in dry_run:"
rostopic info "${HW_TOPIC}" 2>/dev/null || echo "  (not advertised)"

PIDS=()
cleanup() {
  local pid
  for pid in "${PIDS[@]+"${PIDS[@]}"}"; do
    kill "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT

if [[ "${ALSO_ECHO_PREVIEW:-1}" == "1" ]]; then
  echo "[laptop] echoing preview ${PREVIEW_TOPIC}"
  rostopic echo "${PREVIEW_TOPIC}" &
  PIDS+=($!)
fi

echo "[laptop] echoing ${HW_TOPIC}"
rostopic echo "${HW_TOPIC}"
# ################################
# Bash: laptop watch Ranger HW + dry-run preview end
# ################################
