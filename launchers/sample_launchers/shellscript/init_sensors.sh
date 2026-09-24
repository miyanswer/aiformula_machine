#!/bin/bash
set -e

SCRIPT_DIR=$(cd $(dirname $0); pwd)

# CAN
bash ${SCRIPT_DIR}/can_bringup.sh

# IMU (VectorNav)
IMU_PORT="${IMU_PORT:-/dev/ttyUSB0}"
if [ -e "${IMU_PORT}" ]; then
    sudo chmod 666 "${IMU_PORT}"
else
    echo "[WARN] ${IMU_PORT} not found. VectorNav node will keep retrying (reconnect_ms)."
fi
