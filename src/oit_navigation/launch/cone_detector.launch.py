"""
コーン検出 (cone_detector) だけを起動する. 実機でのパラメータ調整用.

    cone_detector  カメラ画像 -> models/cone.pt で検出 -> 地面投影で base_link の位置を推定

パラメータは navigation_params.yaml の cone_detector セクション (six_lane.launch.py / simulator_test.launch.py と同じ).
調整した値はこの YAML に書けば 6レーン走行にもそのまま効く.

例 (実機. 先に bash/1_bringup_hardware.sh で ZED を起動しておく):
    ros2 launch oit_navigation cone_detector.launch.py
    ros2 launch oit_navigation cone_detector.launch.py conf_threshold:=0.3
例 (Web シミュレータ + rosbridge):
    ros2 launch oit_navigation cone_detector.launch.py simulator:=true device:=cpu
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
    """前回起動の cone_detector が残っていると同じトピックに二重に出るので掃除する.
    "cone_detector" だけだとこの launch (cone_detector.launch.py) 自身に一致するので実行ファイルのパスで絞る."""
    try:
        subprocess.run(["pkill", "-9", "-f", "lib/oit_navigation/cone_detector"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    except Exception:
        pass


def _nodes(context):
    get = lambda name: context.launch_configurations[name]  # noqa: E731
    simulator = get("simulator").lower() == "true"
    image_topic = get("image_topic") or (
        get_frame_ids_and_topic_names()[1]["sensing"]["zedx"]["left_image"]["undistorted"]
        + ("/compressed" if simulator else ""))

    params = {"image_topic": image_topic, "model_path": get("model_path"), "device": get("device")}
    if get("conf_threshold"):
        params["conf_threshold"] = float(get("conf_threshold"))
    if simulator:
        params.update({"camera_cx": 960.0, "camera_cy": 540.0})  # シミュレータの理想ピンホール (six_lane.launch.py と同じ)

    return [Node(
        package="oit_navigation", executable="cone_detector", name="cone_detector", output="screen",
        parameters=[get("params_file"), params],
    )]


def generate_launch_description():
    _cleanup_old_processes()
    pkg = get_package_share_directory("oit_navigation")
    args = [
        DeclareLaunchArgument("params_file", default_value=osp.join(pkg, "config", "navigation_params.yaml"),
                              description="cone_detector セクションを持つパラメータ YAML"),
        DeclareLaunchArgument("simulator", default_value="false",
                              description="true: Web シミュレータ (rosbridge) の圧縮画像・理想カメラで動かす"),
        DeclareLaunchArgument("device", default_value="0", description="推論デバイス: '0' (GPU) / 'cpu' / 'mps'"),
        DeclareLaunchArgument("image_topic", default_value="",
                              description="空なら実機の ZED 画像 (simulator:=true なら /compressed)"),
        DeclareLaunchArgument("model_path", default_value=default_workspace_asset("models", "cone.pt")),
        DeclareLaunchArgument("conf_threshold", default_value="",
                              description="空なら params_file の値 (0.4). 調整時に上書きする"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])
