#!/usr/bin/env bash
# ################################
# Bash: laptop dry-run control plane begin
# ################################
# Laptop only. Robot must already run agilex_ws/scripts/run_real_dry_run.sh
# on the same ROS_MASTER_URI.
#
# Usage:
#   export ROS_IP=<laptop_ip>
#   export ROS_MASTER_URI=http://<master>:11311
#   remani_planner/scripts/run_real_dry_run_control_plane.sh
#   remani_planner/scripts/run_real_dry_run_control_plane.sh start_rviz:=false
# ################################
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/_common.sh"
remani_source_ws

RUNNER="${REMANI_WS_ROOT}/run_remani.sh"
if [[ ! -x "${RUNNER}" ]]; then
  echo "ERROR: missing executable ${RUNNER}" >&2
  exit 1
fi

if [[ -z "${ROS_MASTER_URI:-}" ]]; then
  echo "WARN: ROS_MASTER_URI unset; will use a local master (robot drivers won't attach)." >&2
fi

echo "[laptop] ROS_MASTER_URI=${ROS_MASTER_URI:-<local>}"
echo "[laptop] ROS_IP=${ROS_IP:-<unset>}"
echo "[laptop] dry_run control plane: no hardware drivers on this host"

exec "${RUNNER}" mode:=real dry_run:=true \
  start_hardware_drivers:=false \
  use_fake_feedback:=false \
  start_planner:=true \
  start_rviz:=true \
  "$@"
# ################################
# Bash: laptop dry-run control plane end
# ################################
