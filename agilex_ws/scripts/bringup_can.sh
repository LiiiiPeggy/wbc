#!/usr/bin/env bash
# ################################
# Bash: bring up can0 @ 500 kbit/s begin
# ################################
# Usage:
#   sudo ~/agilex_ws/scripts/bringup_can.sh
# or (password prompt once):
#   ~/agilex_ws/scripts/bringup_can.sh
# ################################
set -euo pipefail

IFACE="${CAN_IFACE:-can0}"
BITRATE="${CAN_BITRATE:-500000}"

echo "[bringup_can] ${IFACE} bitrate=${BITRATE}"
if [[ "$(id -u)" -eq 0 ]]; then
  ip link set "${IFACE}" down 2>/dev/null || true
  ip link set "${IFACE}" up type can bitrate "${BITRATE}"
else
  sudo ip link set "${IFACE}" down 2>/dev/null || true
  sudo ip link set "${IFACE}" up type can bitrate "${BITRATE}"
fi

ip -details link show "${IFACE}"
echo "[bringup_can] OK"
# ################################
# Bash: bring up can0 @ 500 kbit/s end
# ################################
