import os.path as osp
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from common_python.workspace_paths import default_workspace_asset


def _cleanup_old_processes():
    """Kill lingering zombie processes from previous launches to prevent accumulation."""
    try:
        subprocess.run(
            ["pkill", "-9", "-f", "yolop_lane_detector|bev_pure_pursuit_node|rviz2|robot_state_publisher|joint_state_publisher"],
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
            default_value=default_workspace_asset("models", "shiho_lane_mask_v2_best.pth"),
            description="Path to YOLOP weight .pth file",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz2 for real-time visualization",
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
            },
        ],
    )

    # 2. BEV Lane Tracker & Pure Pursuit Controller Node
    controller_node = Node(
        package="oit_navigation",
        executable="bev_pure_pursuit_node",
        name="bev_pure_pursuit_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    # 3. RViz2 Visualization (closing RViz automatically shuts down the entire launch)
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
            rviz_node,
        ]
    )
