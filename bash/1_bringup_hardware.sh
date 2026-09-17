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

# Jetsonイメージ(docker/Dockerfile.jetson)では xacro / rosbridge_server 等の
# apt未提供パッケージを /opt/extra_ros_ws にvcs-clone+colconビルドして別ワーク
# スペースとして持たせている(x86版は全て /opt/ros/humble に直接apt導入され
# るため、この行はJetson固有)。これを source しないと、上の
# /opt/ros/humble/setup.bash だけでは xacro (hardware_bringup.launch.py が
# transitiveにimportする) も、下の start_rosbridge() が起動する
# rosbridge_server も見つからずに失敗する。
if [ -f "/opt/extra_ros_ws/install/setup.bash" ]; then
    source /opt/extra_ros_ws/install/setup.bash
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

# ハードウェアBringup Launchの実行
ros2 launch sample_launchers hardware_bringup.launch.py
