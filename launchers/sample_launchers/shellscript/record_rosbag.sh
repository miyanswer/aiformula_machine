#!/bin/bash
<< COMMENTOUT
データ (rosbag) と画像 (H.264 の MP4 動画) を 1 つのコマンドで記録する. 中では record_rosbag_<名前>.sh と
record_rosbag_video.sh <名前> を別プロセスで同時に動かす. Ctrl+C で両方止まる.
画像は Jetson が出す JPEG 版を受けて MP4 にするので, 別 PC (Dell 等) からでも記録できる (要 ffmpeg).
    bash record_rosbag.sh 6lane      # -> rosbag/<日付_時刻>/6lane/{data,video}
    bash record_rosbag.sh qp         # -> rosbag/<日付_時刻>/qp/{data,video}
    bash record_rosbag.sh gamepad    # -> rosbag/<日付_時刻>/gamepad/{data,video}
    CRF=18 bash record_rosbag.sh 6lane   # 動画を高画質にする (既定 23. 小さいほど高画質・大きいファイル)
画像を rosbag で取りたいときは record_rosbag_image.sh を別端末で使う.
COMMENTOUT

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)
source "${SCRIPT_DIR}/record_rosbag_common.sh"   # ROSBAG_ROOT (保存先の親)

name=$1
case "${name}" in
    6lane|qp|gamepad) ;;
    *) echo "使い方: bash record_rosbag.sh <6lane|qp|gamepad>" >&2; exit 1 ;;
esac

# 2 つが同時に保存先を決めると別の秒のディレクトリに分かれうるので, ここで 1 つに決めて渡す
export ROSBAG_ROOT
export RUN_DIR="${ROSBAG_ROOT}/$(date '+%Y%m%d_%H%M%S')/${name}"

# ジョブ制御を有効にして各記録を別プロセスグループで動かす (無効だと非対話シェルの背景ジョブは
# SIGINT を無視する設定で起動され, ros2 bag record が Ctrl+C で止まらない). Ctrl+C はここで受けて両方へ送る
set -m
bash "${SCRIPT_DIR}/record_rosbag_${name}.sh" &
data_pid=$!
bash "${SCRIPT_DIR}/record_rosbag_video.sh" "${name}" &
video_pid=$!
# 止め方: 1 回目 SIGINT (ros2 bag record が bag を閉じて終わる) -> 止まらなければ 2 回目 SIGTERM -> 3 回目 SIGKILL.
# Ctrl+C を押すたびに 1 段進み, 押さなくても 10 秒ごとに自動で次へ進む (固まったまま残らないように)
stop_level=0
stop_children() {
    stop_level=$((stop_level + 1))
    local sig=INT
    case ${stop_level} in
        1) echo "[record_rosbag] 停止中 (bag を閉じています). 止まらなければもう一度 Ctrl+C" >&2 ;;
        2) sig=TERM; echo "[record_rosbag] SIGTERM で止めます. まだ止まらなければもう一度 Ctrl+C" >&2 ;;
        *) sig=KILL; echo "[record_rosbag] SIGKILL で強制終了します (ros2 bag reindex <bag> で復旧できる)" >&2 ;;
    esac
    kill -${sig} -- -${data_pid} -${video_pid} 2>/dev/null
    stop_t=${SECONDS}
}
trap stop_children INT TERM

# どちらかが先に終わった (Ctrl+C / トピック名が読めない等) ら, もう片方も止めて終わる.
# (前景の sleep で待つと Ctrl+C がそちらにだけ届くので, sleep も背景に回して組み込みの wait で待つ)
while [ ${stop_level} -eq 0 ] && kill -0 "${data_pid}" 2>/dev/null && kill -0 "${video_pid}" 2>/dev/null; do
    sleep 1 &
    wait $!
done
[ ${stop_level} -eq 0 ] && stop_children
# 生死はプロセスグループ単位で見る (bash だけ先に終わって ros2 bag record が残ることがあるので)
while kill -0 -- -"${data_pid}" 2>/dev/null || kill -0 -- -"${video_pid}" 2>/dev/null; do
    sleep 1 &
    wait $!
    [ $((SECONDS - stop_t)) -ge 10 ] && stop_children
done
wait
if [ -d "${RUN_DIR}/data" ] && [ -d "${RUN_DIR}/video" ]; then
    echo "[record_rosbag] 保存先: ${RUN_DIR}/{data,video}"
else
    echo "[record_rosbag] 記録できていません (上のエラーを確認): ${RUN_DIR}" >&2
    exit 1
fi
