#!/usr/bin/env bash
# ################################
# Bash: robot-host RoboSense lidar begin
# ################################
# Usage:
#   ~/agilex_ws/scripts/run_lidar.sh
# Env:
#   WITH_RVIZ=1  use upstream start.launch (includes RViz on the robot)
# ################################
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/_common.sh"
remani_source_ws

echo "[run_lidar] ROS_MASTER_URI=${ROS_MASTER_URI:-<local>}"
echo "[run_lidar] expected cloud: /rslidar_points (RSHELIOS_16P)"

if [[ "${WITH_RVIZ:-0}" == "1" ]]; then
  echo "[run_lidar] WITH_RVIZ=1 → rslidar_sdk/start.launch"
  roslaunch rslidar_sdk start.launch "$@"
else
  LAUNCH_FILE="${SCRIPT_DIR}/launch/rslidar_no_rviz.launch"
  echo "[run_lidar] launching ${LAUNCH_FILE}"
  roslaunch "${LAUNCH_FILE}" "$@"
fi
# ################################
# Bash: robot-host RoboSense lidar end
# ################################
