import os.path as osp
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from common_python.workspace_paths import default_workspace_asset

# MP4 動画で信号機距離推定ノードをデバッグするための起動ファイル。
#   MP4 -> video_publisher -> /aiformula_sensing/.../left_image/undistorted/compressed
#        -> traffic_light_distance_node -> /aiformula_perception/traffic_light/*
#        -> rviz2 (annotated_image を表示)

_CAMERA_TOPIC = "/aiformula_sensing/zed_node/left_image/undistorted"


def _cleanup_old_processes():
    """前回起動のゾンビプロセスを掃除する。"""
    try:
        subprocess.run(
            ["pkill", "-9", "-f",
             "video_publisher|traffic_light_distance_node|rviz2"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except Exception:
        pass


def generate_launch_description():
    _cleanup_old_processes()

    pkg_oit_navigation = get_package_share_directory("oit_navigation")
    default_params_file = osp.join(pkg_oit_navigation, "config", "traffic_light_params.yaml")
    default_rviz_file = osp.join(pkg_oit_navigation, "config", "traffic_light_rviz.rviz")

    launch_args = [
        DeclareLaunchArgument(
            "video_path",
            default_value=default_workspace_asset("mp4", "shihou_video_2026_08_24_13_51_47.mp4"),
            description="デバッグに使う MP4 動画の絶対パス",
        ),
        DeclareLaunchArgument(
            "fps",
            default_value="15.0",
            description="動画の再生フレームレート [FPS]",
        ),
        DeclareLaunchArgument(
            "loop",
            default_value="true",
            description="動画を最後まで再生したら先頭へループするか",
        ),
        DeclareLaunchArgument(
            "device",
            default_value="cpu",
            description="推論デバイス: 'cpu' / 'mps' / '0' (CUDA)",
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value=default_workspace_asset("models", "traffic_light.pt"),
            description="YOLO 信号機モデル (.pt) のパス",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="traffic_light_distance_node のパラメータ YAML",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="RViz2 で annotated_image を表示するか",
        ),
    ]

    # 1. 動画配信ノード (MP4 -> カメラ compressed トピック)
    video_publisher_node = Node(
        package="oit_navigation",
        executable="video_publisher",
        name="video_publisher",
        output="screen",
        parameters=[{
            "video_path": LaunchConfiguration("video_path"),
            "topic_name": _CAMERA_TOPIC,
            "frame_id": "zed_left_camera_optical_frame",
            "fps": LaunchConfiguration("fps"),
            "loop": LaunchConfiguration("loop"),
        }],
    )

    # 2. 信号機検出 & 画面占有率による距離逆算ノード
    traffic_light_distance_node = Node(
        package="oit_navigation",
        executable="traffic_light_distance_node",
        name="traffic_light_distance_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "image_topic": _CAMERA_TOPIC + "/compressed",
                "model_path": LaunchConfiguration("model_path"),
                "device": LaunchConfiguration("device"),
                "publish_annotated_image": True,
            },
        ],
    )

    # 3. RViz2 で検出枠 + 推定距離のオーバーレイを目視確認 (閉じるとlaunch全体が終了)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="traffic_light_rviz",
        output="screen",
        arguments=["-d", default_rviz_file],
        condition=IfCondition(LaunchConfiguration("rviz")),
        on_exit=Shutdown(),
    )

    return LaunchDescription(
        launch_args
        + [
            video_publisher_node,
            traffic_light_distance_node,
            rviz_node,
        ]
    )
