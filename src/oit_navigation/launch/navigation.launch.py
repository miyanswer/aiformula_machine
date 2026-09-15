import os.path as osp
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from common_python.launch_util import get_frame_ids_and_topic_names
from common_python.workspace_paths import default_workspace_asset


def _cleanup_old_processes():
    """Kill lingering zombie processes from previous launches to prevent accumulation."""
    try:
        subprocess.run(
            ["pkill", "-9", "-f",
             "yolop_lane_detector|bev_pure_pursuit_node|traffic_light_distance_node|object_publisher_node|image_compressor_node|rviz2|robot_state_publisher|joint_state_publisher"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except Exception:
        pass



def generate_launch_description():
    _cleanup_old_processes()

    TOPIC_NAMES = get_frame_ids_and_topic_names()[1]

    pkg_oit_navigation = get_package_share_directory("oit_navigation")
    default_params_file = osp.join(pkg_oit_navigation, "config", "navigation_params.yaml")
    default_traffic_light_params_file = osp.join(pkg_oit_navigation, "config", "traffic_light_params.yaml")
    default_rviz_file = osp.join(pkg_oit_navigation, "config", "oit_navigation.rviz")

    launch_args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to navigation parameters YAML file",
        ),
        DeclareLaunchArgument(
            "use_device",
            default_value="0",
            description="Inference device: '0' (GPU) or 'cpu'",
        ),
        DeclareLaunchArgument(
            "weight_path",
            default_value=default_workspace_asset("models", "honda_shihou_finetuned_best.pth"),
            description="Path to YOLOP weight .pth file",
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
            default_value=TOPIC_NAMES["sensing"]["zedx"]["left_image"]["undistorted"],
            description="Input camera image topic (raw, from the real ZED X camera)",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz2 for real-time visualization",
        ),
        DeclareLaunchArgument(
            "enable_controller",
            default_value="true",
            description="Launch bev_pure_pursuit_node (disable to run perception-only, e.g. manual/gamepad driving)",
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
        DeclareLaunchArgument(
            "image_compressor",
            default_value="true",
            description="Launch image_compressor_node for the spectator-facing aiformula_pilot feed",
        ),
    ]

    # 1. YOLOP Lane & Road Segmentation Node
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

    # 2. BEV Lane Tracker & Pure Pursuit Controller Node
    controller_node = Node(
        package="oit_navigation",
        executable="bev_pure_pursuit_node",
        name="bev_pure_pursuit_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_controller")),
        parameters=[LaunchConfiguration("params_file")],
    )

    # 3. Traffic Light Detection & Occupancy-Based Distance Estimation Node
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

    # 4. Detected-Rect -> World-Frame ObjectInfo Node
    object_publisher_node = Node(
        package="oit_navigation",
        executable="object_publisher_node",
        name="object_publisher_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("object_publisher")),
        parameters=[LaunchConfiguration("params_file")],
    )

    # 5. Spectator-Facing Compressed Image Feed (aiformula_pilot)
    image_compressor_node = Node(
        package="oit_navigation",
        executable="image_compressor_node",
        name="image_compressor_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("image_compressor")),
        parameters=[{
            "input_topic": LaunchConfiguration("input_image_topic"),
        }],
    )

    # 6. RViz2 Visualization (closing RViz automatically shuts down the entire launch)
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
            yolop_node,
            controller_node,
            traffic_light_distance_node,
            object_publisher_node,
            image_compressor_node,
            rviz_node,
        ]
    )
