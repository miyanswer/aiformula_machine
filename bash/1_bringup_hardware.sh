#!/bin/bash
# ==============================================================================
# 1_bringup_hardware.sh
# 
# [実機専用] 機体・センサー類（CAN, ZED X, IMU, Microstrain, Odometry, Twist Mux）
# のハードウェア初期化とROS2ノードの起動を行います。
# ※PC単体テスト時にはこのスクリプトではなく 2_test_pc_standalone.sh を使用してください。
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ROS 2 環境セットアップ
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
fi

echo "========================================="
echo "  [AI Formula] Initializing Hardware Sensors..."
echo "========================================="

# センサー通信初期化スクリプトの実行（CAN/USB設定等）
INIT_SCRIPT="${SCRIPT_DIR}/../launchers/sample_launchers/shellscript/init_sensors.sh"
if [ -f "${INIT_SCRIPT}" ]; then
    bash "${INIT_SCRIPT}"
else
    echo "[WARN] init_sensors.sh not found at ${INIT_SCRIPT}. Skipping hardware setup."
fi

echo "========================================="
echo "  [AI Formula] Launching Hardware Bringup..."
echo "========================================="

# ハードウェアBringup Launchの実行
ros2 launch sample_launchers hardware_bringup.launch.py
