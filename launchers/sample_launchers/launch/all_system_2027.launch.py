import os.path as osp
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    all_system_2027.launch.py - AI Formula 実機全システム一括起動Launch

    一括起動されるシステム:
      1. 機体ハードウェア基盤 (hardware_bringup: カメラ, IMU, CAN, motor_controller, twist_mux, odom)
      2. 白線検出・周回マップ走行・信号機検出 (oit_navigation/navigation.launch.py:
         lane_detector (YOLOP 既定) + odom_imu_localizer + lane_navigator +
         traffic_light_distance_node + image_compressor_node)
    """
    pkg_sample_launchers = get_package_share_directory("sample_launchers")
    pkg_oit_navigation = get_package_share_directory("oit_navigation")

    launch_args = [
        DeclareLaunchArgument(
            "use_device",
            default_value="0",
            description="Inference device for YOLOP/YOLO: '0' (GPU) or 'cpu'",
        ),
        DeclareLaunchArgument(
            "enable_traffic_light",
            default_value="true",
            description="Launch traffic light depth detector (true/false)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz2 for monitoring (true/false)",
        ),
        DeclareLaunchArgument(
            "backend",
            default_value="yolop",
            description="White-line detector: 'yolop' (models/ YOLOP, default) or 'ufld' (needs UFLD weights)",
        ),
        DeclareLaunchArgument(
            "use_tensorrt",
            default_value="false",
            description="Run YOLOP via TensorRT on Jetson (falls back to PyTorch if unavailable)",
        ),
        DeclareLaunchArgument(
            "map_save_path",
            default_value="",
            description="Save the lap-1 course map (JSON) here",
        ),
        DeclareLaunchArgument(
            "map_load_path",
            default_value="",
            description="Start from a saved course map (lap 2+ raceline driving)",
        ),
    ]

    # 1. 機体ハードウェア基盤一括起動
    hardware_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_sample_launchers, "launch", "hardware_bringup.launch.py"),
        ),
    )

    # 2. 白線検出・周回マップ走行・信号機検出パイプライン
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_oit_navigation, "launch", "navigation.launch.py"),
        ),
        launch_arguments={
            "use_device": LaunchConfiguration("use_device"),
            "traffic_light": LaunchConfiguration("enable_traffic_light"),
            "rviz": LaunchConfiguration("use_rviz"),
            "backend": LaunchConfiguration("backend"),
            "use_tensorrt": LaunchConfiguration("use_tensorrt"),
            "map_save_path": LaunchConfiguration("map_save_path"),
            "map_load_path": LaunchConfiguration("map_load_path"),
        }.items(),
    )

    return LaunchDescription(
        launch_args
        + [
            hardware_bringup,
            navigation,
        ]
    )
