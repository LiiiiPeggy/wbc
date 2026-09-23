#!/usr/bin/env bash
# ################################
# Bash: read-only CR10 status / joints monitor begin
# ################################
# Usage:
#   ~/agilex_ws/scripts/watch_cr10_status.sh
# Env:
#   WATCH_MODE=status|raw_joints|both   (default: both)
# ################################
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/_common.sh"
remani_require_ros

MODE="${WATCH_MODE:-both}"
STATUS_TOPIC="/remani/cr10_status"
RAW_JOINTS="/remani/cr10_joint_states_raw"
PLAN_JOINTS="/remani/cr10_joint_states"

echo "[watch_cr10] Action server (if up):"
rostopic list 2>/dev/null | grep -E 'follow_joint_trajectory|cr10_robot' || true
echo "[watch_cr10] status topic info:"
rostopic info "${STATUS_TOPIC}" 2>/dev/null || echo "  ${STATUS_TOPIC} not advertised (laptop State Bridge publishes this)"
echo "[watch_cr10] raw joints (driver):"
rostopic info "${RAW_JOINTS}" 2>/dev/null || echo "  ${RAW_JOINTS} not advertised yet"

case "${MODE}" in
  status)
    echo "[watch_cr10] echoing ${STATUS_TOPIC}"
    rostopic echo "${STATUS_TOPIC}"
    ;;
  raw_joints)
    echo "[watch_cr10] echoing ${RAW_JOINTS}"
    rostopic echo "${RAW_JOINTS}"
    ;;
  both)
    echo "[watch_cr10] echoing ${RAW_JOINTS} (driver) + ${STATUS_TOPIC} if present"
    echo "--- tip: planned joints ${PLAN_JOINTS} come from laptop State Bridge ---"
    rostopic echo "${RAW_JOINTS}" &
    RAW_PID=$!
    trap 'kill ${RAW_PID} 2>/dev/null || true' EXIT
    # Status may only appear after laptop remani_state_bridge is online.
    if rostopic list 2>/dev/null | grep -qx "${STATUS_TOPIC}"; then
      rostopic echo "${STATUS_TOPIC}"
    else
      echo "[watch_cr10] ${STATUS_TOPIC} absent; holding on raw joints only"
      wait "${RAW_PID}"
    fi
    ;;
  *)
    echo "ERROR: WATCH_MODE must be status|raw_joints|both" >&2
    exit 2
    ;;
esac
# ################################
# Bash: read-only CR10 status / joints monitor end
# ################################
