"""
MP4 動画で白線検出 (左境界/中央線/右境界の割り当てまで) と信号機検出をデバッグする起動ファイル (実機不要).

    MP4 -> video_publisher -> .../left_image/undistorted(/compressed)
        -> lane_detector (backend: yolop / ufld) -> LaneLines, Path x3, 注釈画像
        -> traffic_light_distance_node (traffic_light:=true のとき)
        -> RViz2

動画にはオドメトリ (CAN/IMU) が無いため, 周回マップ作成と QP 走行 (odom_imu_localizer / lane_navigator)
は起動しない. それらは Web シミュレータ (simulator_test.launch.py) か実機で検証する.

例:
    ros2 launch oit_navigation video_test.launch.py backend:=yolop traffic_light:=false
    ros2 launch oit_navigation video_test.launch.py backend:=ufld video_path:=/aiformula_machine/mp4/xxx.mp4
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
    try:
        subprocess.run(
            ["pkill", "-9", "-f",
             "video_publisher|lane_detector|traffic_light_distance_node|rviz2|robot_state_publisher|joint_state_publisher"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False,
        )
    except Exception:
        pass


def generate_launch_description():
    _cleanup_old_processes()
    FRAME_IDS, TOPIC_NAMES = get_frame_ids_and_topic_names()
    camera_topic = TOPIC_NAMES["sensing"]["zedx"]["left_image"]["undistorted"]

    pkg = get_package_share_directory("oit_navigation")
    pkg_vehicle = get_package_share_directory("sample_vehicle")

    args = [
        DeclareLaunchArgument("video_path",
                              default_value=default_workspace_asset("mp4", "shihou_video_2026_08_24_13_51_47.mp4"),
                              description="検証する MP4 のパス"),
        DeclareLaunchArgument("use_device", default_value="cpu", description="'cpu' / '0' (CUDA) / 'mps'"),
        DeclareLaunchArgument("fps", default_value="15.0"),
        DeclareLaunchArgument("loop", default_value="true"),
        DeclareLaunchArgument("backend", default_value="yolop", description="'yolop' / 'ufld'"),
        DeclareLaunchArgument("weight_path",
                              default_value=default_workspace_asset("models", "honda_shihou_finetuned_best.pth"),
                              description="YOLOP の重み (.pth)"),
        DeclareLaunchArgument("ufld_weight_path",
                              default_value=default_workspace_asset("models", "ufld_honda_finetuned_best.pth")),
        DeclareLaunchArgument("use_tensorrt", default_value="false"),
        DeclareLaunchArgument("tensorrt_engine_path", default_value=""),
        DeclareLaunchArgument("lane_width", default_value="3.5", description="中央線 <-> 境界線の距離の初期値 [m]"),
        DeclareLaunchArgument("input_image_topic", default_value=camera_topic + "/compressed"),
        DeclareLaunchArgument("params_file", default_value=osp.join(pkg, "config", "navigation_params.yaml")),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("traffic_light", default_value="true"),
        DeclareLaunchArgument("traffic_light_model_path",
                              default_value=default_workspace_asset("models", "traffic_light.pt")),
        DeclareLaunchArgument("traffic_light_params_file",
                              default_value=osp.join(pkg, "config", "traffic_light_params.yaml")),
    ]

    vehicle_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(osp.join(pkg_vehicle, "launch", "vehicle_tf_broadcaster.launch.py")),
        launch_arguments={"vehicle_name": "ai_car1", "use_sim_time": "false"}.items(),
    )
    video = Node(
        package="oit_navigation", executable="video_publisher", name="video_publisher", output="screen",
        parameters=[{
            "video_path": LaunchConfiguration("video_path"), "topic_name": camera_topic,
            "frame_id": FRAME_IDS["zedx"]["left"], "fps": LaunchConfiguration("fps"),
            "loop": LaunchConfiguration("loop"),
        }],
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
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", osp.join(pkg, "config", "oit_navigation.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")), on_exit=Shutdown(),
    )
    return LaunchDescription(args + [vehicle_tf, video, lane_detector, traffic_light, rviz])
