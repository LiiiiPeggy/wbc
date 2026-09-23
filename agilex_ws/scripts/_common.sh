#!/usr/bin/env bash
# ################################
# Bash: shared remani host helpers begin
# ################################
# Source from sibling scripts. Layout:
#   agilex_ws/
#     scripts/   ← these helpers
#     src/       ← catkin packages
#     build/ devel/
# Robot copy is typically ~/agilex_ws with the same shape.
# ################################

set -euo pipefail

REMANI_SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMANI_WS_ROOT="$(cd "${REMANI_SCRIPTS_DIR}/.." && pwd)"
REMANI_WS_SRC="${REMANI_WS_ROOT}/src"

remani_source_ws() {
  if [[ -f /opt/ros/melodic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/melodic/setup.bash
  elif [[ -f /opt/ros/noetic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
  else
    echo "ERROR: no /opt/ros/{melodic,noetic}/setup.bash found" >&2
    return 1
  fi
  if [[ ! -d "${REMANI_WS_SRC}" ]]; then
    echo "ERROR: missing catkin source dir ${REMANI_WS_SRC}" >&2
    return 1
  fi
  if [[ ! -f "${REMANI_WS_ROOT}/devel/setup.bash" ]]; then
    echo "ERROR: missing ${REMANI_WS_ROOT}/devel/setup.bash — run build_agx_drivers.sh first" >&2
    return 1
  fi
  # shellcheck disable=SC1090
  source "${REMANI_WS_ROOT}/devel/setup.bash"
  export DOBOT_TYPE="${DOBOT_TYPE:-cr10}"
}

remani_require_ros() {
  remani_source_ws
  if ! command -v rostopic >/dev/null 2>&1; then
    echo "ERROR: rostopic not found after sourcing workspace" >&2
    return 1
  fi
  if ! rosnode list >/dev/null 2>&1; then
    echo "ERROR: cannot reach ROS master (set ROS_MASTER_URI / ROS_IP, start roscore)" >&2
    echo "  current ROS_MASTER_URI=${ROS_MASTER_URI:-<unset>}" >&2
    echo "  current ROS_IP=${ROS_IP:-<unset>}" >&2
    return 1
  fi
}

# ################################
# Bash: shared remani host helpers end
# ################################
