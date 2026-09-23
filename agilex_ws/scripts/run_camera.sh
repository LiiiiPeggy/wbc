#!/usr/bin/env bash
# ################################
# Bash: robot-host RealSense D435 begin
# ################################
# Usage:
#   ~/agilex_ws/scripts/run_camera.sh
#   ~/agilex_ws/scripts/run_camera.sh align_depth:=true enable_pointcloud:=true
# ################################
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/_common.sh"
remani_source_ws

echo "[run_camera] ROS_MASTER_URI=${ROS_MASTER_URI:-<local>}"
echo "[run_camera] Intel RealSense via realsense2_camera/rs_camera.launch"
echo "[run_camera] typical topics: /camera/color/image_raw /camera/depth/image_rect_raw"

roslaunch realsense2_camera rs_camera.launch "$@"
# ################################
# Bash: robot-host RealSense D435 end
# ################################
