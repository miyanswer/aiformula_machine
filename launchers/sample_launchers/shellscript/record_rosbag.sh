#!/bin/bash
<< COMMENTOUT
データと画像を 1 つのコマンドで記録する. 中では record_rosbag_<名前>.sh と record_rosbag_image.sh <名前> を
別プロセスで同時に動かすので, 画像の書き込みが小さいトピックの記録を引きずらないのは別端末のときと同じ.
Ctrl+C で両方止まる. 画像は既定で Jetson が出す JPEG 版を取るので, 別 PC (Dell 等) からでも記録できる.
    bash record_rosbag.sh 6lane      # -> ~/rosbag/<日付_時刻>/6lane/{data,image}
    bash record_rosbag.sh qp         # -> ~/rosbag/<日付_時刻>/qp/{data,image}
    bash record_rosbag.sh gamepad    # -> ~/rosbag/<日付_時刻>/gamepad/{data,image}
    RAW=1 bash record_rosbag.sh 6lane              # 画像を JPEG 版ではなく生画像で取る (Jetson 上で記録するとき)
    RECORD_ANNOTATED=1 bash record_rosbag.sh 6lane # 検出器の注釈付き画像も取る
COMMENTOUT

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)

name=$1
case "${name}" in
    6lane|qp|gamepad) ;;
    *) echo "使い方: bash record_rosbag.sh <6lane|qp|gamepad>" >&2; exit 1 ;;
esac

# 2 つが同時に保存先を決めると別の秒のディレクトリに分かれうるので, ここで 1 つに決めて渡す
export RUN_DIR="${HOME}/rosbag/$(date '+%Y%m%d_%H%M%S')/${name}"

# ジョブ制御を有効にして各記録を別プロセスグループで動かす (無効だと非対話シェルの背景ジョブは
# SIGINT を無視する設定で起動され, ros2 bag record が Ctrl+C で止まらない). Ctrl+C はここで受けて両方へ送る
set -m
bash "${SCRIPT_DIR}/record_rosbag_${name}.sh" &
data_pid=$!
bash "${SCRIPT_DIR}/record_rosbag_image.sh" "${name}" &
image_pid=$!
trap 'kill -INT -- -${data_pid} -${image_pid} 2>/dev/null' INT TERM

# どちらかが先に終わった (Ctrl+C / トピック名が読めない等) ら, もう片方も止めて終わる.
# (前景の sleep で待つと Ctrl+C がそちらにだけ届くので, sleep も背景に回して組み込みの wait で待つ)
while kill -0 "${data_pid}" 2>/dev/null && kill -0 "${image_pid}" 2>/dev/null; do
    sleep 1 &
    wait $!
done
kill -INT -- -${data_pid} -${image_pid} 2>/dev/null
wait
if [ -d "${RUN_DIR}/data" ] && [ -d "${RUN_DIR}/image" ]; then
    echo "[record_rosbag] 保存先: ${RUN_DIR}/{data,image}"
else
    echo "[record_rosbag] 記録できていません (上のエラーを確認): ${RUN_DIR}" >&2
    exit 1
fi
