#!/usr/bin/env bash
# ################################
# Bash: whitelist-build Ranger+CR10 drivers begin
# ################################
# Usage (on Ubuntu 18.04 robot host):
#   ~/agilex_ws/scripts/build_agx_drivers.sh
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

WHITELIST="${CATKIN_WHITELIST_PACKAGES:-ugv_sdk;ranger_msgs;ranger_base;dobot_v4_bringup}"
echo "[build_agx_drivers] ws=${REMANI_WS_ROOT}"
echo "[build_agx_drivers] src=${REMANI_WS_SRC}"
echo "[build_agx_drivers] whitelist=${WHITELIST}"

# Clear whitelist cache when packages change: rm -rf build devel
catkin_make -C "${REMANI_WS_ROOT}" --source "${REMANI_WS_SRC}" \
  -DCATKIN_WHITELIST_PACKAGES="${WHITELIST}" \
  -DCMAKE_BUILD_TYPE=Release

echo "[build_agx_drivers] OK — next: source ${REMANI_WS_ROOT}/devel/setup.bash"
# ################################
# Bash: whitelist-build Ranger+CR10 drivers end
# ################################
