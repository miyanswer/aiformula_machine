import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from common_python.workspace_paths import default_workspace_asset


def generate_launch_description():
    pkg_oit_navigation = get_package_share_directory("oit_navigation")
    default_params_file = osp.join(pkg_oit_navigation, "config", "traffic_light_params.yaml")

    launch_args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to traffic light distance params YAML file",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/aiformula_sensing/zed_node/left_image/undistorted/compressed",
            description="Input camera image topic (Image or CompressedImage)",
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value=default_workspace_asset("models", "traffic_light.pt"),
            description="Path to YOLO traffic light model (.pt)",
        ),
        DeclareLaunchArgument(
            "device",
            default_value="cpu",
            description="Inference device: 'cpu', 'mps', or '0' (CUDA)",
        ),
    ]

    traffic_light_distance_node = Node(
        package="oit_navigation",
        executable="traffic_light_distance_node",
        name="traffic_light_distance_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "model_path": LaunchConfiguration("model_path"),
                "device": LaunchConfiguration("device"),
            },
        ],
    )

    return LaunchDescription(launch_args + [traffic_light_distance_node])
