#!/bin/bash
<< COMMENTOUT
周回マップ + QP (navigation.launch.py の lane_navigator) の rosbag. ~/rosbag/<日付_時刻>/qp/data に保存
画像は別端末で: bash record_rosbag_image.sh qp   (-> ~/rosbag/<日付_時刻>/qp/image)
    bash record_rosbag_qp.sh
共通 (record_rosbag_common.sh): ZED IMU, vectornav IMU, CAN, gyro odom, /tf, /tf_static, twist_mux 出力
再生: ros2 bag play ~/rosbag/<日付_時刻>/qp/data --clock   (画像は別端末で ros2 bag play ~/rosbag/<日付_時刻>/qp/image)
      rviz2 -d $(ros2 pkg prefix oit_navigation)/share/oit_navigation/config/oit_navigation.rviz --ros-args -p use_sim_time:=true
COMMENTOUT

source "$(cd "$(dirname "$0")"; pwd)/record_rosbag_common.sh"

# 認識 + 自己位置 -> QP の判断 -> 指令 + RViz 用の境界・レーシングライン (判断パネル画像は record_rosbag_image.sh)
topics=(
    $(read_yaml "['sensing']['odometry']['odom_imu']")       # lane_navigator が使った自己位置 (周回マップの座標)
    $(read_yaml "['perception']['lane_detector']['lane_lines']")
    $(read_yaml "['perception']['lane_lines']['left']")
    $(read_yaml "['perception']['lane_lines']['center']")
    $(read_yaml "['perception']['lane_lines']['right']")
    $(read_yaml "['perception']['cone_detector']['cones']")
    $(read_yaml "['perception']['cone_detector']['status']")
    $(read_yaml "['perception']['traffic_light']['red_distance']")
    $(read_yaml "['perception']['traffic_light']['green_distance']")
    $(read_yaml "['perception']['traffic_light']['status']")
    $(read_yaml "['control']['lane_navigator_status']")
    $(read_yaml "['control']['traffic_light_stop_status']")
    $(read_yaml "['control']['speed_command']['mpc']")
    $(read_yaml "['control']['speed_command']['gamepad']")      # 手動介入したタイミング
    $(read_yaml "['visualization']['lane_navigator']['left_boundary']")
    $(read_yaml "['visualization']['lane_navigator']['right_boundary']")
    $(read_yaml "['visualization']['lane_navigator']['target_trajectory']")
    $(read_yaml "['visualization']['lane_navigator']['markers']")
    $(read_yaml "['visualization']['cone_detector']['markers']")
    $(read_yaml "['visualization']['traffic_light_stop_markers']")
)
record_bag qp data "${COMMON_TOPICS[@]}" "${topics[@]}"
