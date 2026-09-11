import os.path as osp
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    """
    all_system_2027.launch.py - AI Formula OIT 2027 実機全システム一括起動Launch
    
    一括起動されるシステム:
      1. 機体ハードウェア基盤 (hardware_bringup: カメラ, IMU, CAN, motor_controller, twist_mux, odom)
      2. YOLOP 道路・白線セグメンテーション (object_road_detector)
      3. 2027 2D BEV レーン追従 ＆ Pure Pursuit 制御 (ai_formula_oit_2027: bev_lane_tracker)
      4. 2027 信号機垂直奥行き推定 (ai_formula_oit_2027: traffic_light_detector)
      5. RViz2 リアルタイム監視 [オプション]
    """
    pkg_sample_launchers = get_package_share_directory("sample_launchers")
    pkg_2027 = get_package_share_directory("ai_formula_oit_2027")
    pkg_road_detector = get_package_share_directory("object_road_detector")

    default_2027_params = osp.join(pkg_2027, "config", "controller_params.yaml")
    default_tl_params = osp.join(pkg_2027, "config", "traffic_light_params.yaml")

    launch_args = [
        DeclareLaunchArgument(
            "use_device",
            default_value="0",
            description="Inference device for YOLOP/YOLO: '0' (GPU) or 'cpu'",
        ),
        DeclareLaunchArgument(
            "enable_traffic_light",
            default_value="true",
            description="Launch traffic light depth detector (true/false)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz2 for monitoring (true/false)",
        ),
        DeclareLaunchArgument(
            "controller_params",
            default_value=default_2027_params,
            description="Path to BEV controller params YAML",
        ),
    ]

    # 1. 機体ハードウェア基盤一括起動
    hardware_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_sample_launchers, "launch", "hardware_bringup.launch.py"),
        ),
    )

    # 2. YOLOP 道路・白線セグメンテーション
    object_road_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_road_detector, "launch", "object_road_detector.launch.py"),
        ),
        launch_arguments={
            "use_device": LaunchConfiguration("use_device"),
        }.items(),
    )

    # 3. 2027 BEV レーン追従 ＆ Pure Pursuit コントローラー
    bev_lane_tracker = Node(
        package="ai_formula_oit_2027",
        executable="bev_lane_tracker",
        name="bev_lane_tracker",
        output="screen",
        parameters=[LaunchConfiguration("controller_params")],
        remappings=[
            # twist_mux の自律走行ポート (mpc) にリマップ
            ("/aiformula_control/handle_controller/cmd_vel", "/aiformula_control/extremum_seeking_mpc/cmd_vel"),
        ],
    )

    # 4. 2027 信号機垂直奥行き推定ノード
    traffic_light_detector = Node(
        package="ai_formula_oit_2027",
        executable="traffic_light_detector",
        name="traffic_light_detector",
        output="screen",
        parameters=[
            default_tl_params,
            {
                "image_topic": "/aiformula_sensing/zed_node/left_image/undistorted",
                "device": LaunchConfiguration("use_device"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("enable_traffic_light")),
    )

    # 5. RViz2 可視化
    rviz_config = osp.join(pkg_2027, "config", "bev_control.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        launch_args
        + [
            hardware_bringup,
            object_road_detector,
            bev_lane_tracker,
            traffic_light_detector,
            rviz_node,
        ]
    )
