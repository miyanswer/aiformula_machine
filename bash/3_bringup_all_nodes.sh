#!/bin/bash
# ==============================================================================
# 3_bringup_all_nodes.sh
# 
# [実機フルシステム起動] 実車上でハードウェア初期化（CAN/ZED/IMU等）と
# 自律走行システム（YOLOPレーン認識、BEV追従制御、信号機認識、Twist Mux等）
# の全ノードを一括起動します。
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
echo "  [AI Formula] Launching Full System (Hardware + 2027 Autonomous Stack)..."
echo "========================================="

# rosbridge WebSocket (port 9090) - lets the Mac web_simulator drive this
# vehicle over the LAN via WASD (see web_simulator/index.html's rosbridge
# URL field). Idempotent so re-running this script doesn't double-launch it.
start_rosbridge() {
    if ! pgrep -f "rosbridge_websocket" > /dev/null; then
        echo "[INFO] Starting rosbridge_server (port 9090) for remote (Mac) teleop..."
        ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/rosbridge.log 2>&1 &
        sleep 1
    fi
}

start_rosbridge

# 実機全ノード一括起動Launchの実行
ros2 launch sample_launchers all_system_2027.launch.py \
    use_device:=cuda \
    rviz:=false
