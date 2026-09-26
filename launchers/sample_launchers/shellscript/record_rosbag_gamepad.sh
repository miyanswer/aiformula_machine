#!/bin/bash
<< COMMENTOUT
ゲームパッドで手動走行したときの rosbag. rosbag/<日付_時刻>/gamepad/data に保存
    bash record_rosbag_gamepad.sh
画像は別端末で: bash record_rosbag_image.sh gamepad   (-> rosbag/<日付_時刻>/gamepad/image)
共通 (record_rosbag_common.sh): ZED IMU, vectornav IMU, CAN (車輪の実 RPM), gyro odom, /tf, /tf_static, twist_mux 出力

手動で 1 周してオドメトリのずれ (スリップ) を調べる用に, odom_imu_localizer (CAN 車輪速 + vectornav IMU の積算,
QP が使う自己位置) を記録中だけ起動して一緒に取る. ジャイロバイアスを停止中に推定するので, 記録を始めたら
走り出す前に数秒止まっておくこと.
モーター指令 (motor_controller -> CAN 0x210 の左右 目標 RPM) も取るので, 実 RPM (vehicle_info id=1809) と比べられる.
再生: ros2 bag play rosbag/<日付_時刻>/gamepad/data --clock
COMMENTOUT

source "$(cd "$(dirname "$0")"; pwd)/record_rosbag_common.sh"

oit_prefix=$(ros2 pkg prefix oit_navigation)
start_bg_node "oit_navigation/odom_imu_localizer" \
    "${oit_prefix}/lib/oit_navigation/odom_imu_localizer" --ros-args \
    --params-file "${oit_prefix}/share/oit_navigation/config/navigation_params.yaml"

topics=(
    $(read_yaml "['control']['joy']['gamepad']")                 # スティック入力
    $(read_yaml "['control']['speed_command']['gamepad']")       # それを変換した速度指令
    $(read_yaml "['control']['output_can_data']")                # 左右の目標 RPM (CAN 0x210)
    $(read_yaml "['sensing']['odometry']['odom_imu']")            # CAN + vectornav IMU の自己位置
)

record_bag gamepad data "${COMMON_TOPICS[@]}" "${topics[@]}"
