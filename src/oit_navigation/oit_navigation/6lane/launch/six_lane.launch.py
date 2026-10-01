"""
6レーン動的選択走行 (地図なし・オドメトリなし) の起動ファイル.

    lane_detector      カメラ -> 左境界/中央線/右境界 + 検出点群 (navigation_params.yaml の設定をそのまま使う)
    six_lane_planner   白線 + CAN 車輪速 -> 仮想6レーン -> NN でレーン選択 -> cmd_vel (twist_mux "autonomous")
                       赤信号までの距離 -> 信号機の 5〜10m 手前で停止 (utils/traffic_light_stop.py)
    traffic_light_distance_node  traffic_light.pt で赤/青信号を検出し距離を推定 (traffic_light:=false で無効)
    cone_detector      cone.pt でコーンを検出し位置を推定 -> six_lane_planner が塞がれたレーンを避ける (cone_detector:=false で無効)
    rviz2              config/six_lane.rviz: 仮想6レーン・目標レーン・確率・コーン・信号・判断パネル (rviz:=false で無効)
    six_lane_panel_compressor  判断パネルの JPEG 版 (.../six_lane_planner/panel/compressed, 別 PC で記録・表示する用)

    image_compressor_node  観客向け aiformula_pilot 圧縮映像 (image_compressor:=false で無効. simulator:=true では起動しない)

速度上限 (手動切替): speed_limit:=<m/s> (既定 1.5). 走行中は /aiformula_control/six_lane_planner/speed_limit (Float64)
前方注視点などは現在の車速に合わせて自動補正される (six_lane_core.effective_control). enable_controller:=false なら認識だけ起動する.

例 (実機):
    ros2 launch oit_navigation six_lane.launch.py use_device:=0 use_tensorrt:=true
例 (Web シミュレータ + rosbridge. シミュレータで「6レーン (地図なし)」+「ROS2連携」を選ぶ):
    ros2 launch oit_navigation six_lane.launch.py simulator:=true use_device:=cpu
"""

import os.path as osp
import subprocess

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from common_python.launch_util import get_frame_ids_and_topic_names
from common_python.workspace_paths import default_workspace_asset


def _cleanup_old_processes():
    """前回起動のゾンビプロセスを掃除する."""
    try:
        # 実行ファイルのパス (lib/oit_navigation/<名前>) で探す: 名前だけだと, 同名の引数 (cone_detector:=false 等) を
        # 含むこの ros2 launch 自身のコマンドラインにも一致して自分を kill -9 してしまう.
        # 判断パネルの圧縮は実行ファイルが bringup の zed_image_compressor と同じなのでノード名で探す
        subprocess.run(["pkill", "-9", "-f",
                        "lib/oit_navigation/(lane_detector|six_lane_planner|traffic_light_distance_node|cone_detector)|"
                        "__node:=six_lane_panel_compressor|__node:=image_compressor_node|rviz2"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    except Exception:
        pass


def _has_package(name: str) -> bool:
    """rviz2 が入っていない環境 (画面なしの Jetson イメージなど) では RViz を起動しない (起動すると launch ごと落ちる)."""
    try:
        get_package_prefix(name)
        return True
    except PackageNotFoundError:
        print(f"[six_lane.launch] パッケージ '{name}' が無いので RViz は起動しません")
        return False


def _nodes(context):
    get = lambda name: context.launch_configurations[name]  # noqa: E731
    simulator = get("simulator").lower() == "true"
    pkg = get_package_share_directory("oit_navigation")
    image_topic = get("input_image_topic") or (
        get_frame_ids_and_topic_names()[1]["sensing"]["zedx"]["left_image"]["undistorted"]
        + ("/compressed" if simulator else ""))

    detector_params = {
        "use_device": get("use_device"),
        "weight_path": get("weight_path"),
        "use_tensorrt": get("use_tensorrt").lower() == "true",  # 文字列ではなく bool で渡す (型が違うと lane_detector が落ちる)
        "input_image_topic": image_topic,
        "lane_width": float(get("lane_width")),
        # 発進位置の横ずれ (中央線から, 左正). 中央線の上と仮定すると右端発進で右白線を中央線と取り違える
        "tracker_init_offset": float(get("init_offset")),
    }
    # カメラの内部パラメータはシミュレータも実機と同じ (web_simulator の車載カメラは実機 ZED X の camera_info で描画)
    # なので, navigation_params.yaml の値をそのまま使う
    lane_detector = Node(
        package="oit_navigation", executable="lane_detector", name="lane_detector", output="screen",
        parameters=[osp.join(pkg, "config", "navigation_params.yaml"), detector_params],
    )
    planner = Node(
        package="oit_navigation", executable="six_lane_planner", name="six_lane_planner", output="screen",
        parameters=[
            osp.join(get_package_share_directory("sample_vehicle"), "config", "wheel.yaml"),
            get("six_lane_params_file"),
            # 速度上限 (手動切替). シミュレータの旋回上限は web_simulator/js/vehicle_physics.js MAX_ANGULAR
            {"speed_limit": float(get("speed_limit"))},
            {"max_angular_speed": 1.2} if simulator else {},
        ],
    )
    # 判断パネル画像 (約 5MB/s の生画像) の JPEG 版. 別 PC で記録・表示するときはこちらを使う (購読者がいる間だけ変換)
    panel_topic = get_frame_ids_and_topic_names()[1]["visualization"]["six_lane_planner"]["panel"]
    panel_compressor = Node(
        package="oit_navigation", executable="image_compressor_node", name="six_lane_panel_compressor", output="screen",
        parameters=[{"input_topic": panel_topic, "output_topic": panel_topic + "/compressed", "jpeg_quality": 80}],
    )
    nodes = [lane_detector, panel_compressor]
    if get("enable_controller").lower() == "true":
        nodes.append(planner)
    if get("image_compressor").lower() == "true" and not simulator:
        nodes.append(Node(
            package="oit_navigation", executable="image_compressor_node", name="image_compressor_node",
            output="screen", parameters=[{"input_topic": image_topic}],
        ))
    if get("traffic_light").lower() == "true":
        tl_params = {
            "image_topic": image_topic,
            "model_path": get("traffic_light_model_path"),
            "device": get("use_device"),
            "real_height_m": 0.32,
            "publish_annotated_image": True,
        }
        if simulator:
            # シミュレータの YOLO ボックス用に校正した焦点距離 (幾何的には実機と同じ 733.26px だが, 小さい物体の
            # YOLO ボックスは大きめに出るので停止帯 4〜8m で合うよう校正. web_simulator/js/traffic_light_detector.js と同じ).
            # MyLaps パネルは実機と同じ 32cm 角 (web_simulator/models/MyLaps.obj の Panel_Body)
            tl_params.update({"focal_length_y": 864.7, "reference_image_height": 1080})
        nodes.append(Node(
            package="oit_navigation", executable="traffic_light_distance_node", name="traffic_light_distance_node",
            output="screen", parameters=[get("traffic_light_params_file"), tl_params],
        ))
    if get("cone_detector").lower() == "true":
        cone_params = {"image_topic": image_topic, "model_path": get("cone_model_path"), "device": get("use_device")}
        nodes.append(Node(
            package="oit_navigation", executable="cone_detector", name="cone_detector", output="screen",
            parameters=[osp.join(pkg, "config", "navigation_params.yaml"), cone_params],
        ))
    if get("rviz").lower() == "true" and _has_package("rviz2"):
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
        DeclareLaunchArgument("speed_limit", default_value="1.5",
                              description="速度上限 [m/s] (0.3〜3.0). 前方注視点などは自動で補正される. 走行中は speed_limit トピックで変更可"),
        DeclareLaunchArgument("enable_controller", default_value="true",
                              description="six_lane_planner (自律走行) を起動する. false なら認識だけ"),
        DeclareLaunchArgument("image_compressor", default_value="true",
                              description="観客向け aiformula_pilot 圧縮映像を配信する"),
        DeclareLaunchArgument("rviz", default_value="true", description="RViz2 (config/six_lane.rviz) を起動する"),
        DeclareLaunchArgument("traffic_light", default_value="true",
                              description="信号機検出 (赤信号で停止) を起動する"),
        DeclareLaunchArgument("traffic_light_model_path",
                              default_value=default_workspace_asset("models", "traffic_light.pt")),
        DeclareLaunchArgument("traffic_light_params_file",
                              default_value=osp.join(pkg, "config", "traffic_light_params.yaml")),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])
