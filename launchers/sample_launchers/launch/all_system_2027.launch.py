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
      2. 認識・追従・信号機検出・障害物検出 (oit_navigation/navigation.launch.py:
         yolop_lane_detector + bev_pure_pursuit_node + traffic_light_distance_node +
         object_publisher_node + image_compressor_node)
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
    ]

    # 1. 機体ハードウェア基盤一括起動
    hardware_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_sample_launchers, "launch", "hardware_bringup.launch.py"),
        ),
    )

    # 2. 認識・追従・信号機・障害物検出パイプライン
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_oit_navigation, "launch", "navigation.launch.py"),
        ),
        launch_arguments={
            "use_device": LaunchConfiguration("use_device"),
            "traffic_light": LaunchConfiguration("enable_traffic_light"),
            "rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    return LaunchDescription(
        launch_args
        + [
            hardware_bringup,
            navigation,
        ]
    )
