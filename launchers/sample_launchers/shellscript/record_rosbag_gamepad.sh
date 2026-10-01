#!/bin/bash
<< COMMENTOUT
ゲームパッドで手動走行したときの rosbag. rosbag/<日付_時刻>/gamepad/data に保存
    bash record_rosbag_gamepad.sh
画像は別端末で: bash record_rosbag_image.sh gamepad   (-> rosbag/<日付_時刻>/gamepad/image)
共通 (record_rosbag_common.sh): ZED IMU, vectornav IMU, CAN (車輪の実 RPM), gyro odom, /tf, /tf_static, twist_mux 出力

手動で一定距離を走ってスリップ (車輪速のずれ) を調べる用: 共通トピックの gyro odom (車輪速 + IMU の Yaw) の距離と実測を比べる.
モーター指令 (motor_controller -> CAN 0x210 の左右 目標 RPM) も取るので, 実 RPM (vehicle_info id=1809) と比べられる.
再生: ros2 bag play rosbag/<日付_時刻>/gamepad/data --clock
COMMENTOUT

source "$(cd "$(dirname "$0")"; pwd)/record_rosbag_common.sh"

topics=(
    $(read_yaml "['control']['joy']['gamepad']")                 # スティック入力
    $(read_yaml "['control']['speed_command']['gamepad']")       # それを変換した速度指令
    $(read_yaml "['control']['output_can_data']")                # 左右の目標 RPM (CAN 0x210)
)

record_bag gamepad data "${COMMON_TOPICS[@]}" "${topics[@]}"
