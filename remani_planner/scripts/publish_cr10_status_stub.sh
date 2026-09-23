#!/usr/bin/env bash
# ################################
# Bash: temporary Cr10Status stub (laptop) begin
# ################################
# Laptop only. Fills /remani/cr10_status until a formal AGX RobotStatus
# → remani_real_msgs/Cr10Status bridge exists.
#
# Usage:
#   remani_planner/scripts/publish_cr10_status_stub.sh
# ################################
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/_common.sh"
remani_require_ros

RATE="${CR10_STATUS_RATE:-10}"
echo "[laptop] publishing stub /remani/cr10_status at ${RATE} Hz (Ctrl-C to stop)"
echo "[laptop] connected=true enabled=true error_status=0 robot_mode=5"

rostopic pub -r "${RATE}" /remani/cr10_status remani_real_msgs/Cr10Status \
  "{header: {stamp: now}, connected: true, enabled: true, error_status: 0, robot_mode: 5}"
# ################################
# Bash: temporary Cr10Status stub (laptop) end
# ################################
