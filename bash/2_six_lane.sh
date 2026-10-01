#!/bin/bash
# ==============================================================================
# 2_six_lane.sh
#
# [実機] 6レーン走行 (白線検出 + コーン検出 + 信号機検出 + six_lane_planner) を起動します。
# 先に別端末で bash/1_bringup_hardware.sh (機体ハードウェア) を起動しておくこと。
#
# 速度上限 (手動で切り替え。既定 1.5 m/s):
#     bash bash/2_six_lane.sh 2.0            # 起動時に 2.0 m/s
#     SPEED_LIMIT=2.0 bash bash/2_six_lane.sh
#   走行中に変えるとき (別端末):
#     ros2 topic pub --once /aiformula_control/six_lane_planner/speed_limit std_msgs/msg/Float64 "{data: 2.0}"
#   前方注視点などは現在の車速に合わせて自動で補正されます (src/oit_navigation/oit_navigation/6lane/README.md)。
#
# その他の launch 引数はそのまま渡せます:
#     bash bash/2_six_lane.sh 1.5 rviz:=true use_tensorrt:=false
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi
# Jetson イメージの追加ワークスペース (xacro / rosbridge 等. x86 では存在しない)
if [ -f "/opt/extra_ros_ws/install/setup.bash" ]; then
    source /opt/extra_ros_ws/install/setup.bash
fi
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
fi

# 第 1 引数が数値なら速度上限 [m/s]
if [[ "${1:-}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    SPEED_LIMIT="$1"
    shift
fi
SPEED_LIMIT="${SPEED_LIMIT:-1.5}"

# 推論デバイスは USE_DEVICE で上書き可 (0=GPU / cpu). Jetson (L4T) では TensorRT を既定で使う
USE_DEVICE="${USE_DEVICE:-0}"
USE_TENSORRT="${USE_TENSORRT:-false}"
if [ -f /etc/nv_tegra_release ]; then
    USE_TENSORRT="${USE_TENSORRT:-true}"
fi

echo "========================================="
echo "  [AI Formula] six_lane: speed_limit=${SPEED_LIMIT} m/s device=${USE_DEVICE} tensorrt=${USE_TENSORRT}"
echo "========================================="

ros2 launch oit_navigation six_lane.launch.py \
    speed_limit:="${SPEED_LIMIT}" \
    use_device:="${USE_DEVICE}" \
    use_tensorrt:="${USE_TENSORRT}" \
    rviz:=false \
    "$@"
