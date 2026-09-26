#!/bin/bash
<< COMMENTOUT
画像だけの rosbag. record_rosbag_{6lane,qp,gamepad}.sh (画像以外) とは別プロセスで, 同じ名前を付けて起動する
(record_rosbag.sh <名前> なら両方を 1 コマンドで起動する).
    bash record_rosbag_image.sh 6lane      # -> ~/rosbag/<日付_時刻>/6lane/image   (ZED 左画像 + 6レーンの判断パネル)
    bash record_rosbag_image.sh qp         # -> ~/rosbag/<日付_時刻>/qp/image      (ZED 左画像 + QP の判断パネル)
    bash record_rosbag_image.sh gamepad    # -> ~/rosbag/<日付_時刻>/gamepad/image (ZED 左画像)

既定は Jetson が出している JPEG 版 (.../compressed) を記録する. 別 PC (Dell 等) で記録するときはこれを使うこと:
生画像 (640x360 BGRA, 約 0.9MB x 15Hz = 110Mbps) は LAN 越しの DDS では数 Hz しか届かず, Jetson の CPU も食う.
JPEG 版は hardware_bringup (カメラ: zed_image_compressor) と six_lane / navigation launch (判断パネル) が出す.
    RAW=1 bash record_rosbag_image.sh 6lane               # 生画像で取る (Jetson 上で記録するとき. 約 14MB/s)
    RECORD_ANNOTATED=1 bash record_rosbag_image.sh 6lane  # 検出器の注釈付き画像 (生画像) も取る. 別 PC では重い
再生: データと同時に別端末で  ros2 bag play ~/rosbag/<日付_時刻>/6lane/image   (--clock はデータ側の play だけに付ける)
JPEG の bag から判断をやり直すときは launch に input_image_topic:=/aiformula_sensing/zed_node/left_image/undistorted/compressed
COMMENTOUT

source "$(cd "$(dirname "$0")"; pwd)/record_rosbag_common.sh"

name=$1
case "${name}" in
    6lane|qp|gamepad) ;;
    *) echo "使い方: bash record_rosbag_image.sh <6lane|qp|gamepad>" >&2; exit 1 ;;
esac

if [ "${RAW:-0}" = "1" ]; then
    topics=($(read_yaml "['sensing']['zedx']['left_image']['undistorted']"))
    panel_key=panel
else
    topics=($(read_yaml "['sensing']['zedx']['left_image']['compressed']"))
    panel_key=panel_compressed
fi
case "${name}" in
    6lane) topics+=($(read_yaml "['visualization']['six_lane_planner']['${panel_key}']")) ;;
    qp) topics+=($(read_yaml "['visualization']['lane_navigator']['${panel_key}']")) ;;
esac
if [ "${RECORD_ANNOTATED:-0}" = "1" ]; then
    topics+=(
        $(read_yaml "['visualization']['lane_detector_annotated_image']")
        $(read_yaml "['visualization']['traffic_light_annotated_image']")
        $(read_yaml "['visualization']['cone_detector']['annotated_image']")
    )
fi

record_bag "${name}" image "${topics[@]}"
