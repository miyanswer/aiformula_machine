import os.path as osp
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    hardware_bringup.launch.py - AI Formula 機体ハードウェア基盤一括起動Launch
    
    起動する基盤ノード:
      1. 車両モデル & TF座標系 (sample_vehicle)
      2. ZED X ステレオカメラ (zed_wrapper)
      3. VectorNav IMU/GNSS
      4. CAN 通信ブリッジ (socket_can_bridge: can0 ↔ ROS 2)
      5. モーターコントローラー (motor_controller: cmd_vel ↔ CAN 0x210)
      6. 安全優先度マルチプレクサ (twist_mux: 非常停止・手動介入)
      7. ゲームパッド手動操縦 (gamepad_joy & gamepad_teleop)
      8. 車輪速・ジャイロオドメトリ (odometry_publisher)
      9. 後輪ポテンショメータ (rear_potentiometer)
    """
    VEHICLE_NAME = "ai_car1"

    pkg_sample_launchers = get_package_share_directory("sample_launchers")
    pkg_sample_vehicle = get_package_share_directory("sample_vehicle")
    pkg_motor_controller = get_package_share_directory("motor_controller")

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

    # 2. ZED X カメラ
    zed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_sample_launchers, "launch", "zedx_camera.launch.py"),
        ),
    )

    # 3. VectorNav IMU / GNSS
    vectornav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_sample_launchers, "launch", "vectornav.launch.py"),
        ),
    )

    # 4. ゲームパッド ジョイスティック入力
    gamepad_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_sample_launchers, "launch", "gamepad_joy.launch.py"),
        ),
    )

    # 5. ゲームパッド 速度指令出力 (手動介入用)
    gamepad_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_sample_launchers, "launch", "gamepad_teleop.launch.py"),
        ),
    )

    # 6. Twist Mux (安全優先度切替)
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_sample_launchers, "launch", "twist_mux.launch.py"),
        ),
        launch_arguments={
            "use_rviz": "false",
            "use_runtime_monitor": "false",
        }.items(),
    )

    # 7. モーターコントローラー (cmd_vel → RPM → CAN 0x210)
    motor_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_motor_controller, "launch", "motor_controller.launch.py"),
        ),
    )

    # 8. CAN 通信ブリッジ (can0)
    can_receiver_and_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(pkg_sample_launchers, "launch", "socket_can_bridge.launch.py"),
        ),
    )

    # 9. ジャイロオドメトリ
    gyro_odometry_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("odometry_publisher"), "launch", "gyro_odometry_publisher.launch.py"),
        ),
        launch_arguments={"use_rviz": "false"}.items(),
    )

    # 10. 後輪ポテンショメータ
    rear_potentiometer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("rear_potentiometer"), "launch", "rear_potentiometer.launch.py"),
        ),
    )

    return LaunchDescription([
        vehicle_tf_broadcaster,
        zed_node,
        vectornav,
        gamepad_joy,
        gamepad_teleop,
        twist_mux,
        motor_controller,
        can_receiver_and_sender,
        gyro_odometry_publisher,
        rear_potentiometer,
    ])
