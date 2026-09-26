#!/bin/bash
# record_rosbag_{6lane,qp,gamepad,image}.sh から source する共通部分 (単体では実行しない).
#   - COMMON_TOPICS: 6lane/qp/gamepad すべてに入れるセンサ + 最終指令 (画像は record_rosbag_image.sh で別に取る)
#   - record_bag <名前> <data|image> <トピック...>: ~/rosbag/<日付_時刻>/<名前>/<data|image> に記録.
#       データと画像を別端末で 2 分以内に起動すれば同じ <日付_時刻>/<名前> の下に揃う (起動順は問わない)
#   - start_bg_node <pgrep パターン> <コマンド...>: 記録中だけ動かす補助ノード (未起動なら起動, 終了時に止める)

SHELLSCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)
topic_list_yaml_path="${SHELLSCRIPT_DIR}/../config/topic_list.yaml"
read_yaml() {
    node_path=$1
    python3 -c "import yaml; print(yaml.safe_load(open('$topic_list_yaml_path'))${node_path})" 2>/dev/null
}

COMMON_TOPICS=(
    $(read_yaml "['sensing']['zedx']['imu']")
    $(read_yaml "['sensing']['vectornav']['imu']")
    $(read_yaml "['sensing']['input_can_data']")                  # 車輪の実回転数 (id=1809: 右/左 RPM) もこの中
    $(read_yaml "['sensing']['odometry']['gyro']")
    /tf
    /tf_static
    $(read_yaml "['control']['speed_command']['multiplexed']")
)

BG_PIDS=()
start_bg_node() {
    local pattern=$1
    shift
    if pgrep -f "${pattern}" > /dev/null; then
        echo "[record_rosbag] ${pattern} は起動済み"
        return
    fi
    echo "[record_rosbag] 記録中だけ起動: $*"
    "$@" > /dev/null 2>&1 &
    BG_PIDS+=($!)
    trap 'kill "${BG_PIDS[@]}" 2>/dev/null' EXIT
}

record_bag() {
    local name=$1
    local kind=$2
    shift 2
    # read_yaml が失敗 (PyYAML なし等) するとトピック名が空になり /tf だけの bag になるので止める
    if [ -z "$(read_yaml "['sensing']['input_can_data']")" ]; then
        echo "[record_rosbag] ${topic_list_yaml_path} を読めません (python3 -c 'import yaml' を確認)" >&2
        exit 1
    fi
    # 保存先 ~/rosbag/<日付_時刻>/<名前>/<data|image>.
    # データと画像は別端末で少しずれて起動するので, 相方が PAIR_WINDOW 秒以内に作った同名の走行
    # (まだ自分の <kind> が無いもの) があればそこに入る. 無ければ今の時刻で新しく作る
    # (record_rosbag.sh から両方を同時に起動するときは RUN_DIR で保存先が渡される)
    local root="${HOME}/rosbag"
    local run_dir="${RUN_DIR:-}"
    local latest
    latest=$(ls -1d "${root}"/*/"${name}" 2>/dev/null | sort | tail -n 1)
    if [ -z "${run_dir}" ] && [ -n "${latest}" ] && [ ! -e "${latest}/${kind}" ]; then
        local mtime
        mtime=$(stat -c %Y "${latest}" 2>/dev/null || stat -f %m "${latest}")
        if [ $(( $(date +%s) - mtime )) -le "${PAIR_WINDOW:-120}" ]; then
            run_dir="${latest}"
        fi
    fi
    if [ -z "${run_dir}" ]; then
        run_dir="${root}/$(date '+%Y%m%d_%H%M%S')/${name}"
        while [ -e "${run_dir}" ]; do   # 同じ秒に 2 本起動したとき (ros2 bag record は既存ディレクトリに書けない)
            sleep 1
            run_dir="${root}/$(date '+%Y%m%d_%H%M%S')/${name}"
        done
    fi
    mkdir -p "${run_dir}"
    echo "[record_rosbag] -> ${run_dir}/${kind}"
    ros2 bag record -o "${run_dir}/${kind}" \
        --qos-profile-overrides-path "${SHELLSCRIPT_DIR}/../config/qos_setting.yaml" \
        "$@"
}
