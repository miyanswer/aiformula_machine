#!/bin/bash
# ==============================================================================
# 2_test_pc_standalone.sh
# 
# [PC単体検証用] 実機が手元になくてもPCだけで機能追加やアルゴリズムの動作確認を行うスクリプト。
# MP4動画の再生、YOLOPレーン検出、BEVレーン追従、信号機検出、RViz2による可視化を一括起動します。
#
# 使用例:
#   bash 2_test_pc_standalone.sh
#   bash 2_test_pc_standalone.sh /path/to/custom_video.mp4 cpu
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

# video_path省略時は oit_navigation の launch 側デフォルト
# (workspace内のmp4/を自動探索するdefault_workspace_asset()) を使う。
VIDEO_ARG=""
if [ -n "$1" ]; then
    VIDEO_ARG="video_path:=$1"
fi
DEVICE="${2:-cuda}"

echo "========================================="
echo "  [AI Formula] PC Standalone Test Mode"
echo "  Video Path : ${1:-(launchデフォルト: workspace内のmp4/を自動探索)}"
echo "  Device     : ${DEVICE}"
echo "  RViz2      : Enabled"
echo "========================================="

# Launchの実行
ros2 launch oit_navigation video_test.launch.py \
    ${VIDEO_ARG} \
    use_device:="${DEVICE}" \
    rviz:=true
