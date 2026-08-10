#!/usr/bin/env bash
# ################################
# Bash: REMANI sim launch with tee logging begin
# ################################
# 启动 remani_sim.launch，并将 stdout/stderr 同时输出到终端和日志文件。
# 用法:
#   ./run_remani.sh
#   ./run_remani.sh robot_model:=fast_armer
#   ./run_remani.sh robot_model:=ranger_cr10 target_type:=2
# ################################

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/src/REMANI-Planner/logs"
mkdir -p "${LOG_DIR}"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/run_${TIMESTAMP}.log"

echo "[run_remani] log file: ${LOG_FILE}"
echo "[run_remani] starting: roslaunch remani_planner remani_sim.launch $*"

# stdout 与 stderr 一并 tee 到日志，终端仍实时显示
roslaunch remani_planner remani_sim.launch "$@" 2>&1 | tee "${LOG_FILE}"

# ################################
# Bash: REMANI sim launch with tee logging end
# ################################
