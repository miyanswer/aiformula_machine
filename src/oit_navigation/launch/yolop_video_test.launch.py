import os.path as osp
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from common_python.workspace_paths import default_workspace_asset

# MP4 動画で YOLOP 白線・走路認識ノード単体をデバッグするための起動ファイル。
# Pure Pursuit 制御は含めず、白線検出結果 (annotated_image) だけを確認する。
#   MP4 -> video_publisher -> .../left_image/undistorted/compressed
#        -> yolop_lane_detector -> annotated_image (白線・走路オーバーレイ)
#        -> rviz2

_CAMERA_TOPIC = "/aiformula_sensing/zed_node/left_image/undistorted"


def _cleanup_old_processes():
    """前回起動のゾンビプロセスを掃除する。"""
    try:
        subprocess.run(
            ["pkill", "-9", "-f",
             "video_publisher|yolop_lane_detector|rviz2"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except Exception:
        pass


def generate_launch_description():
    _cleanup_old_processes()

    pkg_oit_navigation = get_package_share_directory("oit_navigation")
    default_params_file = osp.join(pkg_oit_navigation, "config", "navigation_params.yaml")
    default_rviz_file = osp.join(pkg_oit_navigation, "config", "yolop_rviz.rviz")

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
            "use_device",
            default_value="cpu",
            description="推論デバイス: 'cpu' / 'mps' / '0' (CUDA)",
        ),
        DeclareLaunchArgument(
            "weight_path",
            default_value=default_workspace_asset("models", "shiho_lane_mask_v2_best.pth"),
            description="YOLOP 白線認識モデル (.pth) のパス",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="yolop_lane_detector のパラメータ YAML",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="RViz2 で annotated_image (白線オーバーレイ) を表示するか",
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

    # 2. YOLOP 白線・走路セグメンテーションノード (単体)
    yolop_node = Node(
        package="oit_navigation",
        executable="yolop_lane_detector",
        name="yolop_lane_detector",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "input_image_topic": _CAMERA_TOPIC + "/compressed",
                "use_device": LaunchConfiguration("use_device"),
                "weight_path": LaunchConfiguration("weight_path"),
            },
        ],
    )

    # 3. RViz2 で白線検出オーバーレイを目視確認 (閉じるとlaunch全体が終了)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="yolop_rviz",
        output="screen",
        arguments=["-d", default_rviz_file],
        condition=IfCondition(LaunchConfiguration("rviz")),
        on_exit=Shutdown(),
    )

    return LaunchDescription(
        launch_args
        + [
            video_publisher_node,
            yolop_node,
            rviz_node,
        ]
    )
