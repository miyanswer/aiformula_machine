#!/bin/bash
<< COMMENTOUT
<Usage Example>
record_rosbag.sh                      # センサ + 走行判断 (oit_navigation の認識結果・判断・RViz 用マーカー/パネル)
record_rosbag.sh run1                 # ~/rosbag/<日付>/run1 に保存
RECORD_ANNOTATED=1 record_rosbag.sh run1   # 検出器の注釈付き画像 (白線・信号・コーン) も記録 (重い)
再生: ros2 bag play ~/rosbag/<日付>/run1
      rviz2 -d $(ros2 pkg prefix oit_navigation)/share/oit_navigation/config/six_lane.rviz       (6レーン走行)
      rviz2 -d $(ros2 pkg prefix oit_navigation)/share/oit_navigation/config/oit_navigation.rviz (周回マップ + QP)
COMMENTOUT

SCRIPT_DIR=$(cd $(dirname $0); pwd)
topic_list_yaml_path="${SCRIPT_DIR}/../config/topic_list.yaml"
read_yaml() {
    node_path=$1
    python3 -c "import yaml; print(yaml.safe_load(open('$topic_list_yaml_path'))${node_path})" 2>/dev/null
}

bag_name=${1:-test}

date_str=`date '+%Y%m%d'`
bag_dir="${HOME}/rosbag/${date_str}"
mkdir -p ${bag_dir}

# 走行判断の確認用 (oit_navigation). どれも小さい (パネル画像は 5Hz, 820x570 程度)
nav_topics=(
    $(read_yaml "['sensing']['odometry']['odom_imu']")
    $(read_yaml "['perception']['lane_detector']['lane_lines']")
    $(read_yaml "['perception']['lane_lines']['left']")
    $(read_yaml "['perception']['lane_lines']['center']")
    $(read_yaml "['perception']['lane_lines']['right']")
    $(read_yaml "['perception']['traffic_light']['red_distance']")
    $(read_yaml "['perception']['traffic_light']['green_distance']")
    $(read_yaml "['perception']['traffic_light']['status']")
    $(read_yaml "['perception']['cone_detector']['cones']")
    $(read_yaml "['perception']['cone_detector']['status']")
    $(read_yaml "['control']['speed_command']['mpc']")
    $(read_yaml "['control']['speed_command']['multiplexed']")
    $(read_yaml "['control']['lane_navigator_status']")
    $(read_yaml "['control']['six_lane_planner']['status']")
    $(read_yaml "['control']['six_lane_planner']['lane_reseed']")
    $(read_yaml "['control']['traffic_light_stop_status']")
    $(read_yaml "['visualization']['lane_navigator']['left_boundary']")
    $(read_yaml "['visualization']['lane_navigator']['right_boundary']")
    $(read_yaml "['visualization']['lane_navigator']['target_trajectory']")
    $(read_yaml "['visualization']['lane_navigator']['panel']")
    $(read_yaml "['visualization']['lane_navigator']['markers']")
    $(read_yaml "['visualization']['six_lane_planner']['target_path']")
    $(read_yaml "['visualization']['six_lane_planner']['markers']")
    $(read_yaml "['visualization']['six_lane_planner']['panel']")
    $(read_yaml "['visualization']['cone_detector']['markers']")
    $(read_yaml "['visualization']['traffic_light_stop_markers']")
)
annotated_topics=()
if [ "${RECORD_ANNOTATED:-0}" = "1" ]; then
    annotated_topics=(
        $(read_yaml "['visualization']['lane_detector_annotated_image']")
        $(read_yaml "['visualization']['traffic_light_annotated_image']")
        $(read_yaml "['visualization']['cone_detector']['annotated_image']")
    )
fi

ros2 bag record -o "${bag_dir}"/"${bag_name}" \
    --qos-profile-overrides-path ${SCRIPT_DIR}/../config/qos_setting.yaml \
    $(read_yaml "['sensing']['zedx']['left_image']['undistorted']") \
    $(read_yaml "['sensing']['zedx']['imu']") \
    $(read_yaml "['sensing']['input_can_data']") \
    $(read_yaml "['sensing']['odometry']['gyro']") \
    $(read_yaml "['sensing']['rear_potentiometer']") \
    /tf \
    /tf_static \
    ${nav_topics[@]} \
    ${annotated_topics[@]}
