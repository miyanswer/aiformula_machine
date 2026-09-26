#!/bin/bash
<< COMMENTOUT
画像だけの rosbag. record_rosbag_{6lane,qp,gamepad}.sh (画像以外) とは別の端末で, 同じ名前を付けて起動する.
画像 (約 14MB/s) を別プロセスに分けると, 小さいトピックの記録が画像の書き込みに引きずられない.
    bash record_rosbag_image.sh 6lane             # -> ~/rosbag/<日付_時刻>/6lane/image   (ZED 左画像 + 6レーンの判断パネル)
    bash record_rosbag_image.sh qp                # -> ~/rosbag/<日付_時刻>/qp/image      (ZED 左画像 + QP の判断パネル)
    bash record_rosbag_image.sh gamepad           # -> ~/rosbag/<日付_時刻>/gamepad/image (ZED 左画像)
    JPEG=1 bash record_rosbag_image.sh 6lane      # 生画像の代わりに JPEG (品質 JPEG_QUALITY=90) で取る. 約 1/10 以下
    RECORD_ANNOTATED=1 bash record_rosbag_image.sh 6lane   # 検出器の注釈付き画像 (白線・信号・コーン) も取る
再生: データと同時に別端末で  ros2 bag play ~/rosbag/<日付_時刻>/6lane/image
      (--clock はデータ側の play だけに付ける)
JPEG で取った bag から判断をやり直すときは launch に input_image_topic:=<生画像>/compressed を渡す.
COMMENTOUT

source "$(cd "$(dirname "$0")"; pwd)/record_rosbag_common.sh"

name=$1
case "${name}" in
    6lane|qp|gamepad) ;;
    *) echo "使い方: bash record_rosbag_image.sh <6lane|qp|gamepad>" >&2; exit 1 ;;
esac

raw_image=$(read_yaml "['sensing']['zedx']['left_image']['undistorted']")
if [ "${JPEG:-0}" = "1" ]; then
    # 記録用の JPEG をこのスクリプトの間だけ出す (観戦用の image_compressor_node とは別名・別トピック)
    oit_prefix=$(ros2 pkg prefix oit_navigation)
    start_bg_node "__node:=rosbag_image_compressor" \
        "${oit_prefix}/lib/oit_navigation/image_compressor_node" --ros-args \
        -r __node:=rosbag_image_compressor \
        -p input_topic:="${raw_image}" \
        -p output_topic:="${raw_image}/compressed" \
        -p jpeg_quality:="${JPEG_QUALITY:-90}"
    topics=("${raw_image}/compressed")
else
    topics=("${raw_image}")
fi

# 判断パネル (5Hz, 820x420 程度の bgr8 = 約 5MB/s) も画像なのでこちらで取る
case "${name}" in
    6lane) topics+=($(read_yaml "['visualization']['six_lane_planner']['panel']")) ;;
    qp) topics+=($(read_yaml "['visualization']['lane_navigator']['panel']")) ;;
esac
if [ "${RECORD_ANNOTATED:-0}" = "1" ]; then
    topics+=(
        $(read_yaml "['visualization']['lane_detector_annotated_image']")
        $(read_yaml "['visualization']['traffic_light_annotated_image']")
        $(read_yaml "['visualization']['cone_detector']['annotated_image']")
    )
fi

record_bag "${name}" image "${topics[@]}"
