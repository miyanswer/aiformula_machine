import os.path as osp
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    all_robot_nodes.launch.py - 車体に載る全ノードの一括起動Launch

    起動する内容:
      1. 機体ハードウェア基盤一式 (hardware_bringup: 車両TF, ZEDカメラ, IMU/GNSS,
         ゲームパッド, twist_mux, motor_controller, CANブリッジ, オドメトリ, リアポテンショメータ)
      2. 認識・自己位置 (oit_navigation: lane_detector 白線 左/中央/右 (YOLOP 既定),
         odom_imu_localizer, 信号機距離推定, spectator向け圧縮画像配信)
      3. 自律走行制御 (lane_navigator: 1周目 中央線走行+境界記録 -> 2周目以降 QP レーシングライン)
         - autopilot:=true の時のみ有効化。
         false の間はゲームパッドでの手動操縦のみ、認識ノードは常時稼働してログ・可視化用途に使える。
    """
    pkg_sample_launchers = get_package_share_directory("sample_launchers")
    pkg_oit_navigation = get_package_share_directory("oit_navigation")

    launch_args = [
        DeclareLaunchArgument(
            "autopilot",
            default_value="false",
            description="If true, lane_navigator drives the robot autonomously",
        ),
        DeclareLaunchArgument(
            "use_device",
            default_value="0",
            description="Inference device for YOLOP/YOLO: '0' (GPU) or 'cpu'",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
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

    hardware_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_sample_launchers, "launch", "hardware_bringup.launch.py"),
        ),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_oit_navigation, "launch", "navigation.launch.py"),
        ),
        launch_arguments={
            "use_device": LaunchConfiguration("use_device"),
            "enable_controller": LaunchConfiguration("autopilot"),
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
