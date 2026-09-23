#!/usr/bin/env bash
# ################################
# Bash: read-only Ranger cmd_vel monitor begin
# ################################
# Usage:
#   ~/agilex_ws/scripts/watch_ranger_cmd.sh
# Env:
#   ALSO_ECHO_CMD_VEL=1  also watch ordinary /cmd_vel (should stay quiet)
# ################################
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/_common.sh"
remani_require_ros

HW_TOPIC="/remani/hardware/ranger/cmd_vel"
echo "[watch_ranger_cmd] publishers on ${HW_TOPIC}:"
rostopic info "${HW_TOPIC}" 2>/dev/null || echo "  (topic not advertised yet)"
echo "[watch_ranger_cmd] echoing ${HW_TOPIC} (Ctrl-C to stop)"

if [[ "${ALSO_ECHO_CMD_VEL:-0}" == "1" ]]; then
  echo "[watch_ranger_cmd] ALSO echoing /cmd_vel in parallel"
  rostopic echo /cmd_vel &
  CMD_PID=$!
  trap 'kill ${CMD_PID} 2>/dev/null || true' EXIT
fi

rostopic echo "${HW_TOPIC}"
# ################################
# Bash: read-only Ranger cmd_vel monitor end
# ################################
