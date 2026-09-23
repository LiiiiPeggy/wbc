#!/usr/bin/env bash
# ################################
# Bash: whitelist-build lidar + RealSense begin
# ################################
# Separate from Ranger/CR10 so a sensor build failure does not block drivers.
#
# Usage:
#   ~/agilex_ws/scripts/build_agx_sensors.sh
# Note: changing CATKIN_WHITELIST_PACKAGES may require:
#   rm -rf ~/agilex_ws/build ~/agilex_ws/devel
# ################################
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/_common.sh"

if [[ -f /opt/ros/melodic/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/melodic/setup.bash
elif [[ -f /opt/ros/noetic/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
else
  echo "ERROR: ROS setup.bash not found" >&2
  exit 1
fi

WHITELIST="${CATKIN_WHITELIST_PACKAGES:-rslidar_sdk;realsense2_camera}"
echo "[build_agx_sensors] ws=${REMANI_WS_ROOT}"
echo "[build_agx_sensors] src=${REMANI_WS_SRC}"
echo "[build_agx_sensors] whitelist=${WHITELIST}"
echo "[build_agx_sensors] requires librealsense2 on the robot host"

catkin_make -C "${REMANI_WS_ROOT}" --source "${REMANI_WS_SRC}" \
  -DCATKIN_WHITELIST_PACKAGES="${WHITELIST}" \
  -DCMAKE_BUILD_TYPE=Release

echo "[build_agx_sensors] OK — re-source ${REMANI_WS_ROOT}/devel/setup.bash"
echo "[build_agx_sensors] To rebuild Ranger/CR10 next, wipe build/devel or reset whitelist."
# ################################
# Bash: whitelist-build lidar + RealSense end
# ################################
