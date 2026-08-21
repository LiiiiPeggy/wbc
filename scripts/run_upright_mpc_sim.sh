#!/usr/bin/env bash
# ################################
# upright PyBullet 仿真启动脚本（方案 A 工作空间）
# ################################
set -euo pipefail

WS=/home/gzz/Codes/wbc/upright_ws
CMEEL_LIB=/home/gzz/.local/lib/python3.8/site-packages/cmeel.prefix/lib
CMEEL_PY=${CMEEL_LIB}/python3.8/site-packages

source /opt/ros/noetic/setup.bash
source /home/gzz/Codes/wbc/ocs2_ws/devel/setup.bash
source "${WS}/devel/setup.bash"

# pip pinocchio 需优先于 ocs2_ws 的 hpp-fcl，否则 undefined symbol
export LD_LIBRARY_PATH="${CMEEL_LIB}:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${CMEEL_PY}:${PYTHONPATH:-}"

# ################################
# 可视化：有 DISPLAY 时默认开 PyBullet GUI；无显示器才无头
# 强制无头：UPRIGHT_HEADLESS=1 ./run_upright_mpc_sim.sh
# 强制开窗：UPRIGHT_HEADLESS=0 ./run_upright_mpc_sim.sh
# ################################
export UPRIGHT_NO_IPYTHON="${UPRIGHT_NO_IPYTHON:-1}"
if [[ -z "${UPRIGHT_HEADLESS:-}" ]]; then
  if [[ -n "${DISPLAY:-}" ]]; then
    UPRIGHT_HEADLESS=0
  else
    UPRIGHT_HEADLESS=1
  fi
fi
export UPRIGHT_HEADLESS
if [[ "${UPRIGHT_HEADLESS}" == "1" ]]; then
  export MPLBACKEND="${MPLBACKEND:-Agg}"
fi

# ################################
# 必须在 source 工作空间之后再 rospack，避免调用方未 source 时得到空路径
# 用法：
#   ./run_upright_mpc_sim.sh
#   ./run_upright_mpc_sim.sh thing_demo.yaml          # demos 下相对名
#   ./run_upright_mpc_sim.sh /abs/path/to/config.yaml
# ################################
UPRIGHT_CMD="$(rospack find upright_cmd)"
DEFAULT_CONFIG="${UPRIGHT_CMD}/config/demos/thing_demo.yaml"

if [[ $# -ge 1 && -n "${1}" ]]; then
  if [[ -f "${1}" ]]; then
    CONFIG="${1}"
  elif [[ -f "${UPRIGHT_CMD}/config/demos/${1}" ]]; then
    CONFIG="${UPRIGHT_CMD}/config/demos/${1}"
  else
    echo "config not found: ${1}" >&2
    echo "try: $(basename "$0")  or  $(basename "$0") thing_demo.yaml" >&2
    exit 1
  fi
else
  CONFIG="${DEFAULT_CONFIG}"
fi

cd "${UPRIGHT_CMD}/scripts/simulations"
echo "config: ${CONFIG}"
if [[ "${UPRIGHT_HEADLESS}" == "1" ]]; then
  echo "PyBullet GUI: OFF (headless). To open the 3D window: UPRIGHT_HEADLESS=0 $0 $*"
else
  echo "PyBullet GUI: ON  DISPLAY=${DISPLAY:-unset}"
fi
python3 ./mpc_sim.py --config "${CONFIG}"
