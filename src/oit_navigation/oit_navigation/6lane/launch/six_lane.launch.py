"""
6レーン動的選択走行 (地図なし・オドメトリなし) の起動ファイル.

    lane_detector      カメラ -> 左境界/中央線/右境界 + 検出点群 (navigation_params.yaml の設定をそのまま使う)
    six_lane_planner   白線 + CAN 車輪速 -> 仮想6レーン -> NN でレーン選択 -> cmd_vel (twist_mux "mpc")
                       赤信号までの距離 -> 信号機の 5〜10m 手前で停止 (utils/traffic_light_stop.py)
    traffic_light_distance_node  traffic_light.pt で赤/青信号を検出し距離を推定 (traffic_light:=false で無効)
    cone_detector      cone.pt でコーンを検出し位置を推定 -> six_lane_planner が塞がれたレーンを避ける (cone_detector:=false で無効)
    rviz2              config/six_lane.rviz: 仮想6レーン・目標レーン・確率・コーン・信号・判断パネル (rviz:=false で無効)

周回マップ + QP の navigation.launch.py (lane_navigator) とは同じ cmd_vel に出すので同時に起動しないこと.

例 (実機):
    ros2 launch oit_navigation six_lane.launch.py use_device:=0 use_tensorrt:=true
例 (Web シミュレータ + rosbridge. シミュレータで「6レーン (地図なし)」+「ROS2連携」を選ぶ):
    ros2 launch oit_navigation six_lane.launch.py simulator:=true use_device:=cpu
"""

import os.path as osp
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from common_python.launch_util import get_frame_ids_and_topic_names
from common_python.workspace_paths import default_workspace_asset


def _cleanup_old_processes():
    """前回起動のゾンビプロセスを掃除する."""
    try:
        subprocess.run(["pkill", "-9", "-f", "lane_detector|lane_navigator|six_lane_planner|traffic_light_distance_node|cone_detector|rviz2"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    except Exception:
        pass


def _nodes(context):
    get = lambda name: context.launch_configurations[name]  # noqa: E731
    simulator = get("simulator").lower() == "true"
    pkg = get_package_share_directory("oit_navigation")
    image_topic = get("input_image_topic") or (
        get_frame_ids_and_topic_names()[1]["sensing"]["zedx"]["left_image"]["undistorted"]
        + ("/compressed" if simulator else ""))

    detector_params = {
        "backend": get("backend"),
        "use_device": get("use_device"),
        "weight_path": get("weight_path"),
        "use_tensorrt": get("use_tensorrt"),
        "input_image_topic": image_topic,
        "lane_width": float(get("lane_width")),
        # 発進位置の横ずれ (中央線から, 左正). 中央線の上と仮定すると右端発進で右白線を中央線と取り違える
        "tracker_init_offset": float(get("init_offset")),
    }
    if simulator:
        # シミュレータのカメラ: 理想ピンホール (光軸 = 画像中心). simulator_test.launch.py と同じ
        detector_params.update({"camera_cx": 960.0, "camera_cy": 540.0})
    lane_detector = Node(
        package="oit_navigation", executable="lane_detector", name="lane_detector", output="screen",
        parameters=[osp.join(pkg, "config", "navigation_params.yaml"), detector_params],
    )
    planner = Node(
        package="oit_navigation", executable="six_lane_planner", name="six_lane_planner", output="screen",
        parameters=[
            osp.join(get_package_share_directory("sample_vehicle"), "config", "wheel.yaml"),
            get("six_lane_params_file"),
            # シミュレータの速度・旋回上限 (web_simulator/js/vehicle_physics.js MAX_SPEED / MAX_ANGULAR)
            {"v_max": 1.5, "max_angular_speed": 1.2} if simulator else {},
        ],
    )
    nodes = [lane_detector, planner]
    if get("traffic_light").lower() == "true":
        tl_params = {
            "image_topic": image_topic,
            "model_path": get("traffic_light_model_path"),
            "device": get("use_device"),
            "real_height_m": 0.32,
            "publish_annotated_image": True,
        }
        if simulator:
            # シミュレータのカメラ用に実測校正した焦点距離 (幾何的には 763.17px だが, 小さい物体の YOLO ボックスは
            # 大きめに出るので停止帯 4〜8m で合うよう校正. web_simulator/js/traffic_light_detector.js と同じ).
            # MyLaps パネルは実機と同じ 32cm 角 (web_simulator/models/MyLaps.obj の Panel_Body)
            tl_params.update({"focal_length_y": 900.0, "reference_image_height": 1080})
        nodes.append(Node(
            package="oit_navigation", executable="traffic_light_distance_node", name="traffic_light_distance_node",
            output="screen", parameters=[get("traffic_light_params_file"), tl_params],
        ))
    if get("cone_detector").lower() == "true":
        cone_params = {"image_topic": image_topic, "model_path": get("cone_model_path"), "device": get("use_device")}
        if simulator:
            cone_params.update({"camera_cx": 960.0, "camera_cy": 540.0})  # lane_detector と同じ理想ピンホール
        nodes.append(Node(
            package="oit_navigation", executable="cone_detector", name="cone_detector", output="screen",
            parameters=[osp.join(pkg, "config", "navigation_params.yaml"), cone_params],
        ))
    if get("rviz").lower() == "true":
        nodes.append(Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", osp.join(pkg, "config", "six_lane.rviz")],
        ))
    return nodes


def generate_launch_description():
    _cleanup_old_processes()
    pkg = get_package_share_directory("oit_navigation")
    args = [
        DeclareLaunchArgument("simulator", default_value="false",
                              description="true: Web シミュレータ (rosbridge) の圧縮画像・理想カメラで動かす"),
        DeclareLaunchArgument("use_device", default_value="0", description="推論デバイス: '0' (GPU) / 'cpu' / 'mps'"),
        DeclareLaunchArgument("backend", default_value="yolop", description="白線検出: 'yolop' / 'ufld'"),
        DeclareLaunchArgument("weight_path",
                              default_value=default_workspace_asset("models", "honda_shihou_finetuned_best.pth")),
        DeclareLaunchArgument("use_tensorrt", default_value="false"),
        DeclareLaunchArgument("input_image_topic", default_value="",
                              description="空なら実機の ZED 画像 (simulator:=true なら /compressed)"),
        DeclareLaunchArgument("lane_width", default_value="3.5", description="中央線 <-> 境界線の距離の初期値 [m]"),
        DeclareLaunchArgument("init_offset", default_value="0.0",
                              description="発進位置: 中央線からの横ずれ [m] (左正). 右端レーン (L6) から発進なら -2.9"),
        DeclareLaunchArgument("six_lane_params_file", default_value=osp.join(pkg, "config", "six_lane_params.yaml")),
        DeclareLaunchArgument("cone_detector", default_value="true",
                              description="コーン検出 (six_lane_planner がコーンで塞がれたレーンを避ける) を起動する"),
        DeclareLaunchArgument("cone_model_path", default_value=default_workspace_asset("models", "cone.pt")),
        DeclareLaunchArgument("rviz", default_value="true", description="RViz2 (config/six_lane.rviz) を起動する"),
        DeclareLaunchArgument("traffic_light", default_value="true",
                              description="信号機検出 (赤信号で停止) を起動する"),
        DeclareLaunchArgument("traffic_light_model_path",
                              default_value=default_workspace_asset("models", "traffic_light.pt")),
        DeclareLaunchArgument("traffic_light_params_file",
                              default_value=osp.join(pkg, "config", "traffic_light_params.yaml")),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])
