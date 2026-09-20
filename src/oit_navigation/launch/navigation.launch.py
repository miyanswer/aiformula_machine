"""
実機 (ZED X / CAN / VectorNav) 用: 白線検出 + 周回マップ作成 + QP レーシングライン走行 + 信号機距離推定.

    lane_detector       カメラ -> 左境界/中央線/右境界 (backend: yolop 既定 / ufld)
    odom_imu_localizer  CAN 車輪速 + IMU -> 自己位置
    lane_navigator      1 周目: 中央線トラッキング + 境界記録 -> QP -> 2 周目以降: レーシングライン
                        cmd_vel は twist_mux の "mpc" 入力 (/aiformula_control/extremum_seeking_mpc/cmd_vel)
    traffic_light_distance_node / image_compressor_node / RViz2

例 (Jetson):
    ros2 launch oit_navigation navigation.launch.py use_device:=0 use_tensorrt:=true
    ros2 launch oit_navigation navigation.launch.py map_save_path:=~/aiformula_maps/course.json
"""

import os.path as osp
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from common_python.launch_util import get_frame_ids_and_topic_names
from common_python.workspace_paths import default_workspace_asset


def _cleanup_old_processes():
    """前回起動のゾンビプロセスを掃除する."""
    try:
        subprocess.run(
            ["pkill", "-9", "-f",
             "lane_detector|lane_navigator|odom_imu_localizer|traffic_light_distance_node|image_compressor_node|"
             "rviz2|robot_state_publisher|joint_state_publisher"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False,
        )
    except Exception:
        pass


def generate_launch_description():
    _cleanup_old_processes()
    TOPIC_NAMES = get_frame_ids_and_topic_names()[1]

    pkg = get_package_share_directory("oit_navigation")
    params_file = osp.join(pkg, "config", "navigation_params.yaml")
    wheel_yaml = osp.join(get_package_share_directory("sample_vehicle"), "config", "wheel.yaml")

    args = [
        DeclareLaunchArgument("params_file", default_value=params_file, description="navigation_params.yaml"),
        DeclareLaunchArgument("use_device", default_value="0", description="推論デバイス: '0' (GPU) / 'cpu'"),
        DeclareLaunchArgument("backend", default_value="yolop",
                              description="白線検出: 'yolop' (models/ の YOLOP, 既定) / 'ufld' (UFLD の重みが必要)"),
        DeclareLaunchArgument("weight_path",
                              default_value=default_workspace_asset("models", "honda_shihou_finetuned_best.pth"),
                              description="YOLOP の重み (.pth)"),
        DeclareLaunchArgument("ufld_weight_path",
                              default_value=default_workspace_asset("models", "ufld_honda_finetuned_best.pth"),
                              description="UFLD の重み (.pth, backend:=ufld のとき)"),
        DeclareLaunchArgument("use_tensorrt", default_value="false",
                              description="YOLOP を TensorRT で推論 (Jetson). 使えなければ PyTorch にフォールバック"),
        DeclareLaunchArgument("tensorrt_engine_path", default_value="",
                              description=".engine のパス. 空なら weight_path から自動 (無ければビルド)"),
        DeclareLaunchArgument("input_image_topic",
                              default_value=TOPIC_NAMES["sensing"]["zedx"]["left_image"]["undistorted"],
                              description="カメラ画像トピック"),
        DeclareLaunchArgument("lane_width", default_value="3.5", description="中央線 <-> 境界線の距離の初期値 [m]"),
        DeclareLaunchArgument("enable_controller", default_value="true",
                              description="lane_navigator (自律走行) を起動する. false なら認識のみ"),
        DeclareLaunchArgument("map_save_path", default_value="", description="1 周目のコースマップ保存先 (JSON)"),
        DeclareLaunchArgument("map_load_path", default_value="", description="保存済みマップで 2 周目から開始"),
        DeclareLaunchArgument("rviz", default_value="true", description="RViz2 を起動する"),
        DeclareLaunchArgument("traffic_light", default_value="true", description="信号機距離推定ノードを起動する"),
        DeclareLaunchArgument("traffic_light_model_path",
                              default_value=default_workspace_asset("models", "traffic_light.pt")),
        DeclareLaunchArgument("traffic_light_params_file",
                              default_value=osp.join(pkg, "config", "traffic_light_params.yaml")),
        DeclareLaunchArgument("image_compressor", default_value="true",
                              description="観客向け aiformula_pilot 圧縮映像を配信する"),
    ]

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
        }],
    )
    localizer = Node(
        package="oit_navigation", executable="odom_imu_localizer", name="odom_imu_localizer", output="screen",
        parameters=[wheel_yaml, LaunchConfiguration("params_file")],
    )
    navigator = Node(
        package="oit_navigation", executable="lane_navigator", name="lane_navigator", output="screen",
        condition=IfCondition(LaunchConfiguration("enable_controller")),
        parameters=[LaunchConfiguration("params_file"), {
            "map_save_path": LaunchConfiguration("map_save_path"),
            "map_load_path": LaunchConfiguration("map_load_path"),
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
        }],
    )
    image_compressor = Node(
        package="oit_navigation", executable="image_compressor_node", name="image_compressor_node",
        output="screen", condition=IfCondition(LaunchConfiguration("image_compressor")),
        parameters=[{"input_topic": LaunchConfiguration("input_image_topic")}],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", osp.join(pkg, "config", "oit_navigation.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")), on_exit=Shutdown(),
    )
    return LaunchDescription(args + [lane_detector, localizer, navigator, traffic_light, image_compressor, rviz])
