#!/usr/bin/env bash
# ################################
# Bash: REMANI sim/real launch with tee logging begin
# ################################
# 用法:
#   ./run_remani.sh
#   ./run_remani.sh robot_model:=ranger_cr10
#   ./run_remani.sh mode:=real dry_run:=true start_planner:=true
# ################################

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/src/REMANI-Planner/logs"
mkdir -p "${LOG_DIR}"

RUN_MODE="sim"
MODE_SEEN="false"
FORWARD_ARGS=()
for ARGUMENT in "$@"; do
  if [[ "${ARGUMENT}" == mode:=* ]]; then
    [[ "${MODE_SEEN}" == "false" ]] || {
      echo "mode may be specified once" >&2
      exit 2
    }
    RUN_MODE="${ARGUMENT#mode:=}"
    MODE_SEEN="true"
  else
    FORWARD_ARGS+=("${ARGUMENT}")
  fi
done

case "${RUN_MODE}" in
  sim)
    LAUNCH_PKG="remani_planner"
    LAUNCH_FILE="remani_sim.launch"
    ;;
  real)
    for ARGUMENT in "${FORWARD_ARGS[@]+"${FORWARD_ARGS[@]}"}"; do
      if [[ "${ARGUMENT}" == execution_owner:=internal ]]; then
        echo "mode:=real rejects execution_owner:=internal" >&2
        exit 2
      fi
    done
    LAUNCH_PKG="remani_real"
    LAUNCH_FILE="remani_real.launch"
    ;;
  *)
    echo "unsupported mode: ${RUN_MODE}" >&2
    exit 2
    ;;
esac

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/run_${RUN_MODE}_${TIMESTAMP}.log"

echo "[run_remani] mode=${RUN_MODE}"
echo "[run_remani] log file: ${LOG_FILE}"
echo "[run_remani] starting: roslaunch ${LAUNCH_PKG} ${LAUNCH_FILE} ${FORWARD_ARGS[*]-}"

if [[ "${RUN_MODE}" == "real" ]]; then
  DOBOT_TYPE=cr10 roslaunch "${LAUNCH_PKG}" "${LAUNCH_FILE}" \
    "${FORWARD_ARGS[@]+"${FORWARD_ARGS[@]}"}" 2>&1 | tee "${LOG_FILE}"
else
  roslaunch "${LAUNCH_PKG}" "${LAUNCH_FILE}" \
    "${FORWARD_ARGS[@]+"${FORWARD_ARGS[@]}"}" 2>&1 | tee "${LOG_FILE}"
fi

# ################################
# Bash: REMANI sim/real launch with tee logging end
# ################################
