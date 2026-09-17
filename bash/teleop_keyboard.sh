#!/bin/bash
# ==============================================================================
# teleop_keyboard.sh
# 
# キーボードによる手動操縦ノードを起動します。
# aiformula_control の twist_mux に対して cmd_vel をパブリッシュします。
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ROS 2 環境セットアップ
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Jetsonイメージ(docker/Dockerfile.jetson)では teleop_twist_keyboard 等の
# apt未提供パッケージを /opt/extra_ros_ws にvcs-clone+colconビルドして別ワーク
# スペースとして持たせている(x86版は全て /opt/ros/humble に直接apt導入され
# るため、この行はJetson固有)。これを source しないと、下の
# `ros2 run teleop_twist_keyboard ...` がパッケージを見つけられず失敗する。
if [ -f "/opt/extra_ros_ws/install/setup.bash" ]; then
    source /opt/extra_ros_ws/install/setup.bash
fi

if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
fi

echo "========================================="
echo "  [AI Formula] Teleop Keyboard Controller"
echo "  Target: /aiformula_control/twist_mux/cmd_vel"
echo "========================================="

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args --remap /cmd_vel:=/aiformula_control/twist_mux/cmd_vel
