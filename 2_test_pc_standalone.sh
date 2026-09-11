#!/bin/bash
# ==============================================================================
# 2_test_pc_standalone.sh (ルート直下実行ラッパー)
#
# 実機なしでPC単体で動画入力・認識・追従・信号機・RViz2の動作確認を行うスクリプト。
# - macOS / Docker環境: 自動的に Docker コンテナ内で実行します。
# - Ubuntu / 実機環境: ネイティブの ROS 2 環境で直接実行します。
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS 2 コマンドがネイティブに存在するかチェック (Ubuntu/実機環境)
if command -v ros2 &> /dev/null; then
    echo "========================================="
    echo "  [AI Formula] Running in Native ROS 2"
    echo "========================================="
    bash "${SCRIPT_DIR}/bash/2_test_pc_standalone.sh" "$@"
else
    # macOS 等で Docker 環境を使用する場合
    echo "========================================="
    echo "  [AI Formula] Running inside Docker Container"
    echo "========================================="
    
    # コンテナが起動しているか確認
    if ! docker compose ps --services --filter "status=running" | grep -q "aiformula_machine"; then
        echo "[INFO] Starting Docker container..."
        docker compose up -d
    fi
    
    # video_path省略時は oit_navigation の launch 側デフォルト
    # (workspace内のmp4/を自動探索するdefault_workspace_asset()) を使う。
    VIDEO_ARG=""
    if [ -n "$1" ]; then
        VIDEO_ARG="video_path:=$1"
    fi
    DEVICE="${2:-cpu}"

    echo "  Video : ${1:-(launchデフォルト: workspace内のmp4/を自動探索)}"
    echo "  Device: ${DEVICE}"
    echo "========================================="
    echo "  Web GUI: http://localhost:8080 (ブラウザでRViz2等の画面が確認できます)"
    echo "========================================="

    docker compose exec aiformula_machine bash -c \
        "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch oit_navigation video_test.launch.py ${VIDEO_ARG} use_device:=${DEVICE} rviz:=true"
fi
