#!/usr/bin/env bash
# ################################
# Bash: laptop remani_planner script helpers begin
# ################################
# Workspace root is the parent of scripts/ (i.e. remani_planner/).
# ################################
set -euo pipefail

REMANI_SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMANI_WS_ROOT="$(cd "${REMANI_SCRIPTS_DIR}/.." && pwd)"

remani_source_ws() {
  if [[ -f /opt/ros/noetic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
  elif [[ -f /opt/ros/melodic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/melodic/setup.bash
  else
    echo "ERROR: no /opt/ros/{noetic,melodic}/setup.bash found" >&2
    return 1
  fi
  if [[ ! -f "${REMANI_WS_ROOT}/devel/setup.bash" ]]; then
    echo "ERROR: missing ${REMANI_WS_ROOT}/devel/setup.bash — build remani_planner first" >&2
    return 1
  fi
  # shellcheck disable=SC1090
  source "${REMANI_WS_ROOT}/devel/setup.bash"
  export DOBOT_TYPE="${DOBOT_TYPE:-cr10}"
}

remani_require_ros() {
  remani_source_ws
  if ! command -v rostopic >/dev/null 2>&1; then
    echo "ERROR: rostopic not found" >&2
    return 1
  fi
  if ! rosnode list >/dev/null 2>&1; then
    echo "ERROR: cannot reach ROS master" >&2
    echo "  ROS_MASTER_URI=${ROS_MASTER_URI:-<unset>}" >&2
    echo "  ROS_IP=${ROS_IP:-<unset>}" >&2
    return 1
  fi
}
# ################################
# Bash: laptop remani_planner script helpers end
# ################################
