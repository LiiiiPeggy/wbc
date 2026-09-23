#!/usr/bin/env bash
# ################################
# Bash: robot-host REMANI dry-run driver bringup begin
# ################################
# Robot host (this script): Ranger + CR10 drivers only. Never starts
# remani_planner / remani_real_node (those stay on the laptop).
#
# Prerequisites:
#   1) build_agx_drivers.sh
#   2) bringup_can.sh
#   3) Robot parked near world origin (first odom samples ≤1 mm / 0.1 deg)
#   4) Shared ROS master with laptop, e.g.:
#        # on robot
#        export ROS_IP=<robot_lan_ip>
#        export ROS_MASTER_URI=http://<laptop_or_robot>:11311
#        # on laptop
#        export ROS_IP=<laptop_lan_ip>
#        export ROS_MASTER_URI=http://<same_master>:11311
#
# Then on the LAPTOP (separate terminal):
#   cd /path/to/wbc/remani_planner
#   source devel/setup.bash
#   ./run_remani.sh mode:=real dry_run:=true \
#     start_hardware_drivers:=false \
#     use_fake_feedback:=false \
#     start_planner:=true \
#     start_rviz:=true
#
# Usage on robot:
#   export ROS_IP=...
#   export ROS_MASTER_URI=...
#   ~/agilex_ws/scripts/run_real_dry_run.sh
#   ~/agilex_ws/scripts/run_real_dry_run.sh robotIp:=192.168.5.1 port_name:=can0
# ################################
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/_common.sh"
remani_source_ws

export DOBOT_TYPE="${DOBOT_TYPE:-cr10}"
LAUNCH_FILE="${SCRIPT_DIR}/launch/remani_hardware_drivers.launch"

if [[ ! -f "${LAUNCH_FILE}" ]]; then
  echo "ERROR: missing ${LAUNCH_FILE}" >&2
  exit 1
fi

if [[ -z "${ROS_MASTER_URI:-}" ]]; then
  echo "WARN: ROS_MASTER_URI unset; roslaunch will start a local master." >&2
  echo "      For laptop+robot dry-run, set the same ROS_MASTER_URI on both hosts." >&2
fi

echo "[run_real_dry_run] DOBOT_TYPE=${DOBOT_TYPE}"
echo "[run_real_dry_run] ROS_MASTER_URI=${ROS_MASTER_URI:-<local>}"
echo "[run_real_dry_run] ROS_IP=${ROS_IP:-<unset>}"
echo "[run_real_dry_run] launching ${LAUNCH_FILE}"
echo "[run_real_dry_run] Ranger HW topic: /remani/hardware/ranger/cmd_vel"
echo "[run_real_dry_run] CR10 raw joints: /remani/cr10_joint_states_raw"
echo "[run_real_dry_run] This process does NOT publish motion under dry_run on laptop."

roslaunch "${LAUNCH_FILE}" "$@"
# ################################
# Bash: robot-host REMANI dry-run driver bringup end
# ################################
