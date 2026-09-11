import os.path as osp
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from common_python.workspace_paths import default_workspace_asset


def generate_launch_description():
    """
    pc_standalone_test.launch.py - 実機不要！PC単体でのオフライン自律走行・認識テストLaunch

    実機ハードウェア（CAN, 実カメラ, 実IMU）が手元になくても、
    MP4動画をカメラトピックとして流し、認識・制御・TF・RViz2の全パイプラインをPC単体で検証できます。

    実体は oit_navigation/video_test.launch.py そのもの
    (動画配信 + YOLOP白線検出 + BEVレーン追従 + 信号機距離推定 + 障害物ObjectInfo化)。
    こちらは `sample_launchers` 側の従来インターフェース名を保つための薄いラッパー。
    """
    pkg_oit_navigation = get_package_share_directory("oit_navigation")

    launch_args = [
        DeclareLaunchArgument(
            "video_path",
            default_value=default_workspace_asset("mp4", "shihou_video_2026_08_24_13_51_47.mp4"),
            description="Absolute path to the MP4 video file",
        ),
        DeclareLaunchArgument(
            "use_device",
            default_value="cpu",
            description="Inference device: 'cpu' or '0' (CUDA GPU)",
        ),
        DeclareLaunchArgument(
            "fps",
            default_value="15.0",
            description="Playback frame rate in FPS",
        ),
        DeclareLaunchArgument(
            "loop",
            default_value="true",
            description="Loop video playback when finished",
        ),
        DeclareLaunchArgument(
            "weight_path",
            default_value=default_workspace_asset("models", "shiho_lane_mask_v2_best.pth"),
            description="Path to the YOLOP weight pth file",
        ),
        DeclareLaunchArgument(
            "enable_traffic_light",
            default_value="true",
            description="Run traffic light depth estimation node",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz2 for visualization",
        ),
    ]

    video_test = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_oit_navigation, "launch", "video_test.launch.py"),
        ),
        launch_arguments={
            "video_path": LaunchConfiguration("video_path"),
            "use_device": LaunchConfiguration("use_device"),
            "fps": LaunchConfiguration("fps"),
            "loop": LaunchConfiguration("loop"),
            "weight_path": LaunchConfiguration("weight_path"),
            "traffic_light": LaunchConfiguration("enable_traffic_light"),
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )

    return LaunchDescription(launch_args + [video_test])
