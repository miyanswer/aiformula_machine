"""
MP4 動画で白線検出 (左境界/中央線/右境界の割り当てまで)・コーン検出・信号機検出をデバッグする起動ファイル (実機不要).
verification_gui.py (検証GUI) の各パイプラインもこの launch を使う.

    MP4 -> video_publisher (既定で 640x360 に縮小 = 実機 ZED X の配信画像と同じ) -> .../left_image/undistorted(/compressed)
        -> lane_detector (lane_detector:=true. backend: yolop / ufld, YOLOP の前処理は roi_mode) -> LaneLines, Path x3, 注釈画像
        -> cone_detector (cone_detector:=true) -> コーン位置, 注釈画像
        -> traffic_light_distance_node (traffic_light:=true) -> 信号までの距離, 注釈画像
        -> RViz2 (config/video_test.rviz: 元の動画 Camera (video) と Lane Detector / Cone Detector / Traffic Light の注釈画像)

動画にはオドメトリ (CAN/IMU) が無いため, 周回マップ作成と QP 走行 (odom_imu_localizer / lane_navigator)
は起動しない. それらは Web シミュレータ (simulator_test.launch.py) か実機で検証する.

例:
    ros2 launch oit_navigation video_test.launch.py backend:=yolop traffic_light:=false
    ros2 launch oit_navigation video_test.launch.py backend:=ufld video_path:=/aiformula_machine/mp4/xxx.mp4
    ros2 launch oit_navigation video_test.launch.py lane_detector:=false traffic_light:=false cone_detector:=true   # コーンだけ
    ros2 launch oit_navigation video_test.launch.py roi_mode:=mask_top   # YOLOP の前処理を比べる
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
            # 実行ファイルのパス (lib/oit_navigation/<名前>) で探す: 名前だけだと, 同名の引数 (cone_detector:=false 等) を
            # 含むこの ros2 launch 自身のコマンドラインにも一致して自分を kill -9 してしまう
            ["pkill", "-9", "-f",
             "lib/oit_navigation/(video_publisher|lane_detector|cone_detector|traffic_light_distance_node)|"
             "rviz2|robot_state_publisher|joint_state_publisher"],
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
        DeclareLaunchArgument("image_width", default_value="640",
                              description="動画をこの幅に縮小して配信 (実機 ZED X の配信画像 640x360 と同じにする). 0 で元のまま"),
        DeclareLaunchArgument("image_height", default_value="360"),
        DeclareLaunchArgument("lane_detector", default_value="true", description="白線検出 (lane_detector) を起動する"),
        DeclareLaunchArgument("roi_mode", default_value="crop_bottom",
                              description="YOLOP の前処理: 'crop_bottom' (学習時と同じ, 既定) / 'mask_top' / 'none'"),
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
        DeclareLaunchArgument("cone_detector", default_value="false", description="コーン検出 (cone_detector) を起動する"),
        DeclareLaunchArgument("cone_model_path", default_value=default_workspace_asset("models", "cone.pt")),
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
            "resize_width": LaunchConfiguration("image_width"), "resize_height": LaunchConfiguration("image_height"),
            # 生画像も出す: RViz2 で元の動画を見る用 (Jetson のイメージには compressed を表示する
            # image_transport プラグインが無い). 検出器は従来どおり /compressed を使う
            "publish_raw": True,
        }],
    )
    lane_detector = Node(
        package="oit_navigation", executable="lane_detector", name="lane_detector", output="screen",
        condition=IfCondition(LaunchConfiguration("lane_detector")),
        parameters=[LaunchConfiguration("params_file"), {
            "roi_mode": LaunchConfiguration("roi_mode"),
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
    cone_detector = Node(
        package="oit_navigation", executable="cone_detector", name="cone_detector", output="screen",
        condition=IfCondition(LaunchConfiguration("cone_detector")),
        parameters=[LaunchConfiguration("params_file"), {
            "image_topic": LaunchConfiguration("input_image_topic"),
            "model_path": LaunchConfiguration("cone_model_path"),
            "device": LaunchConfiguration("use_device"),
        }],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", osp.join(pkg, "config", "video_test.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")), on_exit=Shutdown(),
    )
    return LaunchDescription(args + [vehicle_tf, video, lane_detector, cone_detector, traffic_light, rviz])
