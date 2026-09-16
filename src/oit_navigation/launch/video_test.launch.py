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


def _cleanup_old_processes():
    """Kill lingering zombie processes from previous launches to prevent accumulation."""
    try:
        subprocess.run(
            ["pkill", "-9", "-f", "video_publisher|yolop_lane_detector|bev_pure_pursuit_node|traffic_light_distance_node|object_publisher_node|rviz2|robot_state_publisher|joint_state_publisher"],
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
            "video_path",
            default_value=default_workspace_asset("mp4", "shihou_video_2026_08_24_13_51_47.mp4"),
            description="Absolute path to the test MP4 video file",
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
            default_value=default_workspace_asset("models", "honda_shihou_finetuned_best.pth"),
            description="Path to the YOLOP weight .pth file",
        ),
        DeclareLaunchArgument(
            "use_tensorrt",
            default_value="false",
            description="Run YOLOP via a TensorRT engine instead of PyTorch (see export_tensorrt.py). "
                        "Falls back to PyTorch automatically if the engine/TensorRT bindings are missing.",
        ),
        DeclareLaunchArgument(
            "tensorrt_engine_path",
            default_value="",
            description="Path to the .engine file. Empty = weight_path with a .engine extension.",
        ),
        DeclareLaunchArgument(
            "input_image_topic",
            default_value=TOPIC_NAMES["sensing"]["zedx"]["left_image"]["undistorted"] + "/compressed",
            description="Input camera image topic (Raw or Compressed)",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to navigation params YAML",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz2 for visualization",
        ),
        DeclareLaunchArgument(
            "traffic_light",
            default_value="true",
            description="Launch the traffic light distance estimator node",
        ),
        DeclareLaunchArgument(
            "traffic_light_model_path",
            default_value=default_workspace_asset("models", "traffic_light.pt"),
            description="Path to the YOLO traffic light model (.pt)",
        ),
        DeclareLaunchArgument(
            "traffic_light_params_file",
            default_value=default_traffic_light_params_file,
            description="Path to traffic light distance params YAML",
        ),
        DeclareLaunchArgument(
            "object_publisher",
            default_value="true",
            description="Launch the object_publisher_node (converts YOLOP boxes to ObjectInfo)",
        ),
    ]

    # 1. 車両 TF 座標系ブロードキャスター
    vehicle_tf_broadcaster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_sample_vehicle, "launch", "vehicle_tf_broadcaster.launch.py"),
        ),
        launch_arguments={
            "vehicle_name": VEHICLE_NAME,
            "use_sim_time": "false",
        }.items(),
    )

    # 2. 動画配信ノード (MP4 -> カメラトピック)
    video_publisher_node = Node(
        package="oit_navigation",
        executable="video_publisher",
        name="video_publisher",
        output="screen",
        parameters=[{
            "video_path": LaunchConfiguration("video_path"),
            "topic_name": TOPIC_NAMES["sensing"]["zedx"]["left_image"]["undistorted"],
            "frame_id": FRAME_IDS["zedx"]["left"],
            "fps": LaunchConfiguration("fps"),
            "loop": LaunchConfiguration("loop"),
        }],
    )

    # 3. YOLOP 白線・道路セグメンテーションノード
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
                "use_tensorrt": LaunchConfiguration("use_tensorrt"),
                "tensorrt_engine_path": LaunchConfiguration("tensorrt_engine_path"),
                "input_image_topic": LaunchConfiguration("input_image_topic"),
            },
        ],
    )

    # 4. BEV レーン追従 ＆ Pure Pursuit 制御ノード
    bev_controller_node = Node(
        package="oit_navigation",
        executable="bev_pure_pursuit_node",
        name="bev_pure_pursuit_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    # 5. 信号機検出 & 画面占有率による距離逆算ノード (信号は 1 辺 32cm の正方形)
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

    # 6. 検出Rect -> 世界座標ObjectInfo変換ノード
    object_publisher_node = Node(
        package="oit_navigation",
        executable="object_publisher_node",
        name="object_publisher_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("object_publisher")),
        parameters=[LaunchConfiguration("params_file")],
    )

    # 7. RViz2 可視化 (closing RViz shuts down all pipeline nodes cleanly)
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
            video_publisher_node,
            yolop_node,
            bev_controller_node,
            traffic_light_distance_node,
            object_publisher_node,
            rviz_node,
        ]
    )
