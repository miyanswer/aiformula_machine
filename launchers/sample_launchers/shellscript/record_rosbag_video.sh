#!/bin/bash
<< COMMENTOUT
画像を H.264 の MP4 動画で保存する (画像の rosbag の代わり). record_rosbag_{6lane,qp,gamepad}.sh (data) と
同時に, 同じ名前を付けて起動する (record_rosbag.sh <名前> なら両方を 1 コマンドで起動する).
    bash record_rosbag_video.sh 6lane      # -> rosbag/<日付_時刻>/6lane/video/{camera,panel}.mp4
    bash record_rosbag_video.sh qp         # -> rosbag/<日付_時刻>/qp/video/{camera,panel}.mp4
    bash record_rosbag_video.sh gamepad    # -> rosbag/<日付_時刻>/gamepad/video/camera.mp4
Jetson が出している JPEG 版 (.../compressed) を受け, デコードせずに ffmpeg (libx264) で MP4 にする.
  camera.mp4  ZED 左画像 (/aiformula_sensing/zed_node/left_image/undistorted/compressed)
  panel.mp4   判断パネル (6lane / qp のとき)
  *_stamps.csv  フレームごとの header.stamp (ROS 時刻). data の rosbag と時刻を突き合わせる用
必要なもの: ffmpeg (sudo apt install ffmpeg). 画質は CRF=23 (小さいほど高画質) で変えられる
COMMENTOUT

source "$(cd "$(dirname "$0")"; pwd)/record_rosbag_common.sh"

name=$1
case "${name}" in
    6lane|qp|gamepad) ;;
    *) echo "使い方: bash record_rosbag_video.sh <6lane|qp|gamepad>" >&2; exit 1 ;;
esac
check_topic_list
if ! command -v ffmpeg > /dev/null && [ -z "${FFMPEG:-}" ]; then
    echo "[record_rosbag] ffmpeg がありません: sudo apt install ffmpeg" >&2
    exit 1
fi

targets=("camera=$(read_yaml "['sensing']['zedx']['left_image']['compressed']")")
case "${name}" in
    6lane) targets+=("panel=$(read_yaml "['visualization']['six_lane_planner']['panel_compressed']")") ;;
    qp) targets+=("panel=$(read_yaml "['visualization']['lane_navigator']['panel_compressed']")") ;;
esac

run_dir=$(resolve_run_dir "${name}" video)
echo "[record_rosbag] -> ${run_dir}/video"
python3 "${SHELLSCRIPT_DIR}/record_video.py" --out-dir "${run_dir}/video" --crf "${CRF:-23}" "${targets[@]}" < /dev/null
