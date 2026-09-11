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

if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
fi

echo "========================================="
echo "  [AI Formula] Teleop Keyboard Controller"
echo "  Target: /aiformula_control/twist_mux/cmd_vel"
echo "========================================="

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args --remap /cmd_vel:=/aiformula_control/twist_mux/cmd_vel
