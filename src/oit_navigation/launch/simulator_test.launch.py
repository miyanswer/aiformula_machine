import os.path as osp
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from common_python.launch_util import get_frame_ids_and_topic_names
from common_python.workspace_paths import default_workspace_asset

# Web シミュレータ（web_simulator/）と rosbridge_server 経由で連携するための
# 起動ファイル。video_test.launch.py と違い、動画ファイルの代わりに
# ブラウザが配信するカメラ画像を認識・制御パイプラインの入力として使う:
#
#   ブラウザ (web_simulator) --rosbridge--> .../left_image/undistorted/compressed
#     -> yolop_lane_detector (白線・走路セグメンテーション)
#     -> bev_pure_pursuit_node (BEV変換 -> Pure Pursuit 制御)
#     -> .../extremum_seeking_mpc/cmd_vel --rosbridge--> ブラウザ (twist_mux "mpc" ソース)
#
# 事前に `make rosbridge`（または `ros2 launch rosbridge_server
# rosbridge_websocket_launch.xml`）で rosbridge_server を起動し、ブラウザで
# http://localhost:8000/web_simulator/ を開いて「接続」してから、このファイルを
# 起動してください（起動手順は web_simulator/README.md を参照）。
# video_publisher は起動しません -- 画像入力は rosbridge 経由のブラウザです。


def _cleanup_old_processes():
    """前回起動のゾンビプロセスを掃除する（rosbridge_server は対象外）。"""
    try:
        subprocess.run(
            ["pkill", "-9", "-f",
             "yolop_lane_detector|bev_pure_pursuit_node|traffic_light_distance_node|object_publisher_node|rviz2|robot_state_publisher|joint_state_publisher"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except Exception:
        pass


def generate_launch_description():
    _cleanup_old_processes()

    VEHICLE_NAME = "ai_car1"

    FRAME_IDS, TOPIC_NAMES = get_frame_ids_and_topic_names()

    pkg_oit_navigation = get_package_share_directory("oit_navigation")
    pkg_sample_vehicle = get_package_share_directory("sample_vehicle")

    default_params_file = osp.join(pkg_oit_navigation, "config", "navigation_params.yaml")
    default_traffic_light_params_file = osp.join(pkg_oit_navigation, "config", "traffic_light_params.yaml")
    default_rviz_file = osp.join(pkg_oit_navigation, "config", "oit_navigation.rviz")

    launch_args = [
        DeclareLaunchArgument(
            "use_device",
            default_value="cpu",
            description="推論デバイス: 'cpu' / 'mps' / '0' (CUDA GPU)",
        ),
        DeclareLaunchArgument(
            "weight_path",
            default_value=default_workspace_asset("models", "honda_shihou_finetuned_best.pth"),
            description="YOLOP 白線認識モデル (.pth) のパス",
        ),
        DeclareLaunchArgument(
            "input_image_topic",
            # web_simulator/js/simulator.js の IMAGE_TOPIC_NAME と同じ値
            # (rosbridge 経由でブラウザが配信する圧縮カメラ画像)。
            default_value=TOPIC_NAMES["sensing"]["zedx"]["left_image"]["undistorted"] + "/compressed",
            description="入力カメラ画像トピック（既定値はWebシミュレータの配信トピックと一致）",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="navigation_params.yaml のパス",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="RViz2 を起動するか",
        ),
        DeclareLaunchArgument(
            "traffic_light",
            default_value="false",
            description="信号機検出ノードを起動するか（Webシミュレータのコースには信号機がないため既定は無効）",
        ),
        DeclareLaunchArgument(
            "traffic_light_model_path",
            default_value=default_workspace_asset("models", "traffic_light.pt"),
            description="YOLO 信号機モデル (.pt) のパス",
        ),
        DeclareLaunchArgument(
            "traffic_light_params_file",
            default_value=default_traffic_light_params_file,
            description="信号機距離推定パラメータ YAML",
        ),
        DeclareLaunchArgument(
            "object_publisher",
            default_value="false",
            description="object_publisher_node（YOLOP検出枠 -> ObjectInfo変換）を起動するか",
        ),
    ]

    # 1. 車両 TF 座標系ブロードキャスター（RViz2 での可視化用）
    vehicle_tf_broadcaster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_sample_vehicle, "launch", "vehicle_tf_broadcaster.launch.py"),
        ),
        launch_arguments={
            "vehicle_name": VEHICLE_NAME,
            "use_sim_time": "false",
        }.items(),
    )

    # 2. YOLOP 白線・道路セグメンテーションノード
    #    (Webシミュレータがrosbridge経由で配信する圧縮画像を購読する)
    yolop_node = Node(
        package="oit_navigation",
        executable="yolop_lane_detector",
        name="yolop_lane_detector",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "use_device": LaunchConfiguration("use_device"),
                "weight_path": LaunchConfiguration("weight_path"),
                "input_image_topic": LaunchConfiguration("input_image_topic"),
            },
        ],
    )

    # 3. BEV レーン追従 ＆ Pure Pursuit 制御ノード
    #    (extremum_seeking_mpc/cmd_vel をrosbridge経由でWebシミュレータへ配信)
    bev_controller_node = Node(
        package="oit_navigation",
        executable="bev_pure_pursuit_node",
        name="bev_pure_pursuit_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    # 4. 信号機検出 & 距離推定ノード（既定では無効。Webシミュレータに信号機の
    #    3Dモデルを追加した場合に traffic_light:=true で有効化する）
    traffic_light_distance_node = Node(
        package="oit_navigation",
        executable="traffic_light_distance_node",
        name="traffic_light_distance_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("traffic_light")),
        parameters=[
            LaunchConfiguration("traffic_light_params_file"),
            {
                "image_topic": LaunchConfiguration("input_image_topic"),
                "model_path": LaunchConfiguration("traffic_light_model_path"),
                "device": LaunchConfiguration("use_device"),
                "real_height_m": 0.32,
                "publish_annotated_image": True,
            },
        ],
    )

    # 5. 検出Rect -> 世界座標ObjectInfo変換ノード（既定では無効）
    object_publisher_node = Node(
        package="oit_navigation",
        executable="object_publisher_node",
        name="object_publisher_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("object_publisher")),
        parameters=[LaunchConfiguration("params_file")],
    )

    # 6. RViz2 可視化（閉じるとパイプライン全体を終了）
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_file],
        condition=IfCondition(LaunchConfiguration("rviz")),
        on_exit=Shutdown(),
    )

    return LaunchDescription(
        launch_args
        + [
            vehicle_tf_broadcaster,
            yolop_node,
            bev_controller_node,
            traffic_light_distance_node,
            object_publisher_node,
            rviz_node,
        ]
    )
