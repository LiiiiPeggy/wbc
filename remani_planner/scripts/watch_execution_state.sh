#!/usr/bin/env bash
# ################################
# Bash: laptop watch /remani/execution_state begin
# ################################
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/_common.sh"
remani_require_ros

TOPIC="/remani/execution_state"
echo "[laptop] echoing ${TOPIC} (Ctrl-C to stop)"
rostopic info "${TOPIC}" 2>/dev/null || {
  echo "ERROR: ${TOPIC} not advertised — start run_real_dry_run_control_plane.sh first" >&2
  exit 1
}
rostopic echo "${TOPIC}"
# ################################
# Bash: laptop watch /remani/execution_state end
# ################################
