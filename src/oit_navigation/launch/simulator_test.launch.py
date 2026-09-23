"""
Web シミュレータ (web_simulator/) と rosbridge_server 経由で連携する起動ファイル.
実機と同じノード構成 (lane_detector / odom_imu_localizer / lane_navigator) を, ブラウザが配信する
センサ (カメラ画像・CAN 車輪速・IMU) で動かし, cmd_vel をブラウザの twist_mux "mpc" 入力へ返す:

    ブラウザ --rosbridge--> .../left_image/undistorted/compressed -> lane_detector -> LaneLines
    ブラウザ --rosbridge--> /aiformula_sensing/vehicle_info (CAN), /aiformula_sensing/vectornav/imu
                            -> odom_imu_localizer -> odom
    lane_navigator -> .../extremum_seeking_mpc/cmd_vel --rosbridge--> ブラウザ (twist_mux "mpc")
    traffic_light_distance_node -> 赤/青信号までの距離 -> lane_navigator (赤なら 5〜10m 手前で停止)
                   -> status / 境界 / レーシングライン --rosbridge--> ブラウザの周回マップ表示

手順: `make rosbridge` -> ブラウザで http://localhost:8000/web_simulator/ を開き「接続」
     -> このファイルを起動 -> シミュレータの「自動運転」タブで「ROS2連携」を選び「自動運転: ON」.
シミュレータのカメラは歪みなし・光軸中心の理想ピンホールなので camera_cx/cy は画像中心,
コースの中央線 <-> 境界線は約 3.1m (web_simulator/js/simulator.js の SIM_LANE_WIDTH と同じ).
"""

import os.path as osp
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from common_python.launch_util import get_frame_ids_and_topic_names
from common_python.workspace_paths import default_workspace_asset


def _cleanup_old_processes():
    """前回起動のゾンビプロセスを掃除する (rosbridge_server は対象外)."""
    try:
        subprocess.run(
            ["pkill", "-9", "-f",
             "lane_detector|lane_navigator|odom_imu_localizer|traffic_light_distance_node|cone_detector|rviz2|"
             "robot_state_publisher|joint_state_publisher"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False,
        )
    except Exception:
        pass


def generate_launch_description():
    _cleanup_old_processes()
    TOPIC_NAMES = get_frame_ids_and_topic_names()[1]
    pkg = get_package_share_directory("oit_navigation")
    pkg_vehicle = get_package_share_directory("sample_vehicle")

    args = [
        DeclareLaunchArgument("use_device", default_value="cpu", description="'cpu' / 'mps' / '0' (CUDA)"),
        DeclareLaunchArgument("backend", default_value="yolop", description="'yolop' / 'ufld'"),
        DeclareLaunchArgument("weight_path",
                              default_value=default_workspace_asset("models", "honda_shihou_finetuned_best.pth")),
        DeclareLaunchArgument("ufld_weight_path",
                              default_value=default_workspace_asset("models", "ufld_honda_finetuned_best.pth")),
        DeclareLaunchArgument("use_tensorrt", default_value="false"),
        DeclareLaunchArgument("tensorrt_engine_path", default_value=""),
        DeclareLaunchArgument("input_image_topic",
                              # web_simulator/js/simulator.js の IMAGE_TOPIC_NAME と同じ
                              default_value=TOPIC_NAMES["sensing"]["zedx"]["left_image"]["undistorted"] + "/compressed"),
        DeclareLaunchArgument("lane_width", default_value="3.1", description="シミュレータのコース: 約 3.1m"),
        DeclareLaunchArgument("params_file", default_value=osp.join(pkg, "config", "navigation_params.yaml")),
        DeclareLaunchArgument("map_save_path", default_value=""),
        DeclareLaunchArgument("map_load_path", default_value=""),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("traffic_light", default_value="true",
                              description="MyLaps パネル (赤/緑に切り替わる) を信号機として検出し, 赤なら停止する"),
        DeclareLaunchArgument("cone_detector", default_value="true", description="コーン検出ノードを起動する"),
        DeclareLaunchArgument("cone_model_path", default_value=default_workspace_asset("models", "cone.pt")),
        DeclareLaunchArgument("traffic_light_model_path",
                              default_value=default_workspace_asset("models", "traffic_light.pt")),
        DeclareLaunchArgument("traffic_light_params_file",
                              default_value=osp.join(pkg, "config", "traffic_light_params.yaml")),
    ]

    vehicle_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(osp.join(pkg_vehicle, "launch", "vehicle_tf_broadcaster.launch.py")),
        launch_arguments={"vehicle_name": "ai_car1", "use_sim_time": "false"}.items(),
    )
    lane_detector = Node(
        package="oit_navigation", executable="lane_detector", name="lane_detector", output="screen",
        parameters=[LaunchConfiguration("params_file"), {
            "backend": LaunchConfiguration("backend"),
            "use_device": LaunchConfiguration("use_device"),
            "weight_path": LaunchConfiguration("weight_path"),
            "ufld_weight_path": LaunchConfiguration("ufld_weight_path"),
            "use_tensorrt": LaunchConfiguration("use_tensorrt"),
            "tensorrt_engine_path": LaunchConfiguration("tensorrt_engine_path"),
            "input_image_topic": LaunchConfiguration("input_image_topic"),
            "lane_width": LaunchConfiguration("lane_width"),
            # シミュレータのカメラ: 理想ピンホール (光軸 = 画像中心), 縦画角 70.6deg
            "camera_cx": 960.0,
            "camera_cy": 540.0,
        }],
    )
    localizer = Node(
        package="oit_navigation", executable="odom_imu_localizer", name="odom_imu_localizer", output="screen",
        # シミュレータ連携では gyro_odometry_publisher の odom TF が無いので自前で出す (RViz 表示用)
        parameters=[osp.join(pkg_vehicle, "config", "wheel.yaml"), LaunchConfiguration("params_file"),
                    {"publish_tf": True}],
    )
    navigator = Node(
        package="oit_navigation", executable="lane_navigator", name="lane_navigator", output="screen",
        parameters=[LaunchConfiguration("params_file"), {
            "map_save_path": LaunchConfiguration("map_save_path"),
            "map_load_path": LaunchConfiguration("map_load_path"),
            # シミュレータの速度・旋回上限 (web_simulator/js/vehicle_physics.js MAX_SPEED / MAX_ANGULAR)
            "raceline.v_max": 1.5,
            "tracker.max_angular_speed": 1.2,
        }],
    )
    traffic_light = Node(
        package="oit_navigation", executable="traffic_light_distance_node", name="traffic_light_distance_node",
        output="screen", condition=IfCondition(LaunchConfiguration("traffic_light")),
        parameters=[LaunchConfiguration("traffic_light_params_file"), {
            "image_topic": LaunchConfiguration("input_image_topic"),
            "model_path": LaunchConfiguration("traffic_light_model_path"),
            "device": LaunchConfiguration("use_device"),
            "real_height_m": 0.32,
            "publish_annotated_image": True,
            # シミュレータのカメラ用に実測校正した焦点距離 (幾何的には 763.17px だが, 小さい物体の YOLO ボックスは
            # 大きめに出るので停止帯 4〜8m で合うよう校正. web_simulator/js/traffic_light_detector.js と同じ)
            "focal_length_y": 900.0,
            "reference_image_height": 1080,
        }],
    )
    cone_detector = Node(
        package="oit_navigation", executable="cone_detector", name="cone_detector", output="screen",
        condition=IfCondition(LaunchConfiguration("cone_detector")),
        parameters=[LaunchConfiguration("params_file"), {
            "image_topic": LaunchConfiguration("input_image_topic"),
            "model_path": LaunchConfiguration("cone_model_path"),
            "device": LaunchConfiguration("use_device"),
            # シミュレータのカメラ: 理想ピンホール (光軸 = 画像中心). lane_detector と同じ
            "camera_cx": 960.0,
            "camera_cy": 540.0,
        }],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", osp.join(pkg, "config", "oit_navigation.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")), on_exit=Shutdown(),
    )
    return LaunchDescription(args + [vehicle_tf, lane_detector, localizer, navigator, traffic_light, cone_detector, rviz])
