#!/bin/bash
<< COMMENTOUT
6レーン動的選択走行 (six_lane.launch.py) の rosbag. rosbag/<日付_時刻>/6lane/data に保存
画像は別端末で: bash record_rosbag_image.sh 6lane   (-> rosbag/<日付_時刻>/6lane/image)
    bash record_rosbag_6lane.sh
共通 (record_rosbag_common.sh): ZED IMU, vectornav IMU, CAN, gyro odom, /tf, /tf_static, twist_mux 出力
再生: ros2 bag play rosbag/<日付_時刻>/6lane/data --clock   (画像は別端末で ros2 bag play rosbag/<日付_時刻>/6lane/image)
      rviz2 -d $(ros2 pkg prefix oit_navigation)/share/oit_navigation/config/six_lane.rviz --ros-args -p use_sim_time:=true
COMMENTOUT

source "$(cd "$(dirname "$0")"; pwd)/record_rosbag_common.sh"

# 認識 -> 6レーンの判断 -> 指令 + RViz 用マーカー (判断パネル画像は record_rosbag_image.sh)
topics=(
    $(read_yaml "['perception']['lane_detector']['lane_lines']")
    $(read_yaml "['perception']['lane_lines']['left']")
    $(read_yaml "['perception']['lane_lines']['center']")
    $(read_yaml "['perception']['lane_lines']['right']")
    $(read_yaml "['perception']['cone_detector']['cones']")
    $(read_yaml "['perception']['cone_detector']['status']")
    $(read_yaml "['perception']['traffic_light']['red_distance']")
    $(read_yaml "['perception']['traffic_light']['green_distance']")
    $(read_yaml "['perception']['traffic_light']['status']")
    $(read_yaml "['control']['six_lane_planner']['status']")
    $(read_yaml "['control']['six_lane_planner']['lane_reseed']")
    $(read_yaml "['control']['traffic_light_stop_status']")
    $(read_yaml "['control']['speed_command']['mpc']")
    $(read_yaml "['control']['speed_command']['gamepad']")      # 手動介入したタイミング
    $(read_yaml "['visualization']['six_lane_planner']['target_path']")
    $(read_yaml "['visualization']['six_lane_planner']['markers']")
    $(read_yaml "['visualization']['cone_detector']['markers']")
    $(read_yaml "['visualization']['traffic_light_stop_markers']")
)
record_bag 6lane data "${COMMON_TOPICS[@]}" "${topics[@]}"
