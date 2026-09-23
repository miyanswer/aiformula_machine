#!/usr/bin/env python3
"""
lane_navigator_node.py - 周回マップ作成 + QP レーシングライン走行ノード.

    入力: aiformula_interfaces/LaneLines (lane_detector)  - 左境界/中央線/右境界 (base_link)
          nav_msgs/Odometry (odom_imu_localizer)          - 自己位置 (odom)
          geometry_msgs/PoseArray (cone_detector)          - コーン位置 (base_link). 回避・1周目の記憶・2周目の補正に使う
          std_msgs/Float32 赤/青信号までの距離 (traffic_light_distance_node) - 赤なら信号の 5〜10m 手前で停止
    出力: geometry_msgs/Twist -> twist_mux の "mpc" 入力 (/aiformula_control/extremum_seeking_mpc/cmd_vel)
          std_msgs/String (JSON)  状態 (/aiformula_control/lane_tracker/status)
          nav_msgs/Path  記録した左右境界 / レーシングライン (odom, RViz・Web シミュレータ用)
          sensor_msgs/Image  判断パネル (状態・周回・速度・信号を日本語で, /aiformula_visualization/lane_navigator/panel)
          visualization_msgs/MarkerArray  車の上の要約テキスト (base_link, /aiformula_visualization/lane_navigator/markers)

    1 周目 (MAPPING): 中央白線の上をレーントラッキング走行しつつ, 前方 x_rec の左右境界点を
        直線は疎・カーブは密に odom 座標へ記録する. スタート地点に戻ったら 1 周完了
        (オドメトリのドリフトで判定できないときは ~/finish_mapping サービスで手動終了).
    コーン回避 (lane_nav/cone_avoidance.py, web_simulator/js/cone_avoidance.js と同じ):
        毎周期: 前方のコーン群を半径 1.0m の禁止円の外周に沿って抜ける操舵バイアスを指令に足す (反応的回避).
        1 周目: 見たコーンを記憶 -> 周回完了時にコースマップと同じ補正で地図座標のコーン地図にする.
        2 周目以降: レーシングラインをコーン地図から押し出して追従し, 見えたコーンとの照合で自己位置も補正する.
    OPTIMIZING: ループ補正したコースマップから QP で最小曲率ライン (アウト・イン・アウト) を別スレッドで計算.
        その間は中央線走行を続ける. map_save_path が指定されていればマップを JSON で保存.
    2 周目以降 (RACING): レーシングラインを追従. 白線観測をマップに合わせて自己位置を補正.

    map_load_path を指定すると保存済みマップから直接 RACING で始まる (1 周目と同じ位置・向きから発進すること).

アルゴリズム本体は oit_navigation/lane_nav/ (ROS 非依存, web_simulator/js/lane_navigator.js と同一).
"""

import json
import math
import os
import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import Trigger

from aiformula_interfaces.msg import LaneLine, LaneLines
from common_python.workspace_paths import resolve_workspace_asset
from oit_navigation.lane_nav import (
    CourseMap, LaneNavigator, LineFit, NavigatorParams, MAPPING, OPTIMIZING, RACING, TrackedLines, optimize_raceline,
)
from oit_navigation.lane_nav.cone_avoidance import NavigatorConeAvoidance
from oit_navigation.lane_nav.line_tracker import ROLES
from oit_navigation.utils.debug_panel import GRAY, WHITE, YELLOW, JapaneseText, text_panel
from oit_navigation.utils.image_util import cv2_to_imgmsg
from oit_navigation.utils.traffic_light_stop_ros import TrafficLightStopRos
from oit_navigation.utils.viz_markers import CONE, ORANGE, _marker, marker_array, text_marker
from visualization_msgs.msg import Marker

STATE_JA = {"MAPPING": "1周目: 中央線を走りながら左右の境界を記録 (地図作成)",
            "OPTIMIZING": "地図から QP でレーシングラインを計算中 (中央線走行を継続)",
            "RACING": "2周目以降: QP レーシングライン (アウト・イン・アウト) を追従",
            "STOPPED": "停止"}


def lane_line_to_fit(msg: LaneLine) -> Optional[LineFit]:
    if not msg.valid:
        return None
    return LineFit(np.array([msg.c0, msg.c1, msg.c2]), msg.x_min, msg.x_max,
                   len(msg.points_x), inferred=not msg.detected)


class LaneNavigatorNode(Node):
    def __init__(self):
        super().__init__("lane_navigator")
        self.params = self._load_params()
        self.nav = LaneNavigator(self.params, async_optimize=True)

        self.pose = None
        self.v = 0.0
        self.omega = 0.0
        self.s = 0.0
        self._last_odom_pose = None
        self._pending_lines: Optional[TrackedLines] = None
        self._lock = threading.Lock()
        self._opt_thread: Optional[threading.Thread] = None
        self._last_map_pub = 0.0
        self._map_saved = False

        # コーン回避 (シミュレータの周回マップ + QP 走行と同じ 4 段構え)
        self.cones_avoid = NavigatorConeAvoidance(self.cone_avoid_enabled, self.cone_deflect, self.cone_landmark,
                                                  self.cone_landmark_gate)
        self.avoider = self.cones_avoid.avoider
        self._cones = []
        self._cones_t = -1e9
        self._cones_new = False

        if self.map_load_path:
            path = resolve_workspace_asset(self.map_load_path)
            with open(path) as f:
                text = f.read()
            self.nav.load_map(CourseMap.from_json(text))
            self.cones_avoid.cone_map = [tuple(c) for c in json.loads(text).get("cones", [])]
            self.get_logger().info(f"保存済みマップから開始: {path} ({len(self.nav.course_map.left)} 断面, "
                                   f"コーン {len(self.cones_avoid.cone_map)} 個)")

        self.create_subscription(LaneLines, self.lane_lines_topic, self._lines_cb, 5)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        if self.cones_topic:
            self.create_subscription(PoseArray, self.cones_topic, self._cones_cb, 5)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 1)
        self.status_pub = self.create_publisher(String, self.status_topic, 1)
        self.left_pub = self.create_publisher(Path, self.left_boundary_topic, 1)
        self.right_pub = self.create_publisher(Path, self.right_boundary_topic, 1)
        self.raceline_pub = self.create_publisher(Path, self.raceline_topic, 1)
        # 赤信号停止: 最終 cmd_vel に速度上限を掛ける (six_lane_planner と共通, utils/traffic_light_stop.py)
        self.tl_stop = TrafficLightStopRos(self)
        # RViz 用の可視化 (走行後に rosbag を再生して判断を確認する)
        self.panel_pub = self.create_publisher(Image, self.panel_topic, 1)
        self.markers_pub = self.create_publisher(MarkerArray, self.markers_topic, 1)
        self.jt = JapaneseText(self.panel_font_path)
        self._last_panel_t = -1e9
        self.create_service(Trigger, "~/finish_mapping", self._finish_mapping_srv)
        self.create_service(Trigger, "~/reset", self._reset_srv)
        self.create_timer(1.0 / self.control_rate, self._control)
        self._last_t: Optional[float] = None
        self.get_logger().info(
            f"lane_navigator: lines={self.lane_lines_topic} odom={self.odom_topic} -> cmd_vel={self.cmd_vel_topic}")

    # ------------------------------------------------------------------ params
    def _load_params(self) -> NavigatorParams:
        d = self.declare_parameter
        self.lane_lines_topic = d("lane_lines_topic", "/aiformula_perception/lane_detector/lane_lines").value
        self.odom_topic = d("odom_topic", "/aiformula_sensing/odom_imu_localizer/odom").value
        self.cmd_vel_topic = d("cmd_vel_topic", "/aiformula_control/extremum_seeking_mpc/cmd_vel").value
        self.status_topic = d("status_topic", "/aiformula_control/lane_tracker/status").value
        self.left_boundary_topic = d("left_boundary_topic", "/aiformula_visualization/lane_navigator/left_boundary").value
        self.right_boundary_topic = d("right_boundary_topic", "/aiformula_visualization/lane_navigator/right_boundary").value
        self.raceline_topic = d("raceline_topic", "/aiformula_visualization/target_trajectory").value
        self.map_frame_id = d("map_frame_id", "odom").value
        self.control_rate = float(d("control_rate", 20.0).value)
        self.map_save_path = str(d("map_save_path", "").value)
        self.map_load_path = str(d("map_load_path", "").value)
        self.panel_topic = d("panel_topic", "/aiformula_visualization/lane_navigator/panel").value
        # コーン回避 (cone_detector の PoseArray. 空文字ならコーン入力なし)
        self.cones_topic = str(d("cone_avoidance.cones_topic", "/aiformula_perception/cone_detector/cones").value)
        self.cones_timeout = float(d("cone_avoidance.cones_timeout", 0.5).value)
        self.cone_avoid_enabled = bool(d("cone_avoidance.reactive", True).value)          # 毎周期の反応的回避
        self.cone_deflect = bool(d("cone_avoidance.deflect_raceline", True).value)       # 2 周目: ラインを押し出す
        self.cone_landmark = bool(d("cone_avoidance.landmark_correction", True).value)   # 2 周目: 自己位置補正
        self.cone_landmark_gate = float(d("cone_avoidance.landmark_gate", 1.0).value)
        self.markers_topic = d("markers_topic", "/aiformula_visualization/lane_navigator/markers").value
        self.panel_rate = float(d("panel_rate", 5.0).value)          # [Hz] 判断パネル画像 (0 で出さない)
        self.panel_font_path = str(d("panel_font_path", "").value)    # 空なら Noto CJK などを自動で探す

        p = NavigatorParams()
        def load(obj, prefix):
            for name in obj.__dataclass_fields__:
                default = getattr(obj, name)
                if isinstance(default, (int, float, bool)) and not isinstance(default, tuple):
                    val = d(f"{prefix}{name}", default).value
                    setattr(obj, name, type(default)(val))
        load(p.recorder, "recorder.")
        load(p.lap, "lap.")
        load(p.raceline, "raceline.")
        load(p.tracker, "tracker.")
        for name in ("heading_window", "lines_timeout", "stop_decel", "map_matching_gain",
                     "map_matching_max_error", "match_x_min", "match_x_max", "match_x_step",
                     "match_max_step_xy", "match_max_step_yaw", "match_min_lines"):
            default = getattr(p, name)
            setattr(p, name, type(default)(d(name, default).value))
        return p

    # ------------------------------------------------------------------ io
    def _lines_cb(self, msg: LaneLines):
        lines = {r: lane_line_to_fit(getattr(msg, r)) for r in ROLES}
        detected = {r: bool(getattr(msg, r).valid and getattr(msg, r).detected) for r in ROLES}
        with self._lock:
            self._pending_lines = TrackedLines(lines, detected, {}, {
                "left": msg.lane_width_left, "right": msg.lane_width_right})

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        if self._last_odom_pose is not None:
            self.s += math.hypot(pose[0] - self._last_odom_pose[0], pose[1] - self._last_odom_pose[1])
        self._last_odom_pose = pose
        self.pose = pose
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    def _cones_cb(self, msg: PoseArray):
        cones = [(p.position.x, p.position.y) for p in msg.poses]
        with self._lock:
            self._cones, self._cones_t, self._cones_new = cones, self.get_clock().now().nanoseconds * 1e-9, True
            # 1 周目: 見えたコーンをその時点の自己位置で記憶する
            self.cones_avoid.observe(self.nav, self.s, self.pose, cones)

    def _save_cones(self):
        """保存済みのコースマップ JSON にコーン地図も書き足す (map_load_path で再開しても回避できるように)."""
        if not self.map_save_path or not self._map_saved:
            return
        path = os.path.expanduser(self.map_save_path)
        with open(path) as f:
            d = json.load(f)
        d["cones"] = [list(c) for c in self.cones_avoid.cone_map]
        with open(path, "w") as f:
            json.dump(d, f)

    # ------------------------------------------------------------------ control
    def _control(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self._last_t is None else min(now - self._last_t, 0.5)
        self._last_t = now
        if self.pose is None:
            self.get_logger().warning(f"オドメトリ待ち: {self.odom_topic}", throttle_duration_sec=5.0)
            return
        with self._lock:
            lines, self._pending_lines = self._pending_lines, None
            cones = list(self._cones) if now - self._cones_t < self.cones_timeout else []
            cones_new, self._cones_new = self._cones_new, False

        cmd = self.nav.step(now, dt, self.pose, self.v, self.omega, self.s, lines)
        self._maybe_start_optimization()
        # コーン: RACING 開始時にコーン地図を確定してラインを押し出す / 2 周目の自己位置補正 / 反応的回避
        was_racing = self.cones_avoid._prev_state == RACING
        v, omega, created = self.cones_avoid.step(now, dt, self.nav, self.pose, cmd.v, cmd.omega, cones, cones_new,
                                                  self.params.lap, self.params.tracker)
        if self.nav.state == RACING and not was_racing:
            if created:
                self._save_cones()
            ca = self.cones_avoid
            self.get_logger().info(f"コーン地図: {len(ca.cone_map)} 個" + (
                " -> レーシングラインをコーンから押し出して追従" if ca.deflected_points is not None else ""))
        v, omega = self.tl_stop.apply(now, dt, self.v, v, omega)

        tw = Twist()
        tw.linear.x = float(v)
        tw.angular.z = float(omega)
        self.cmd_pub.publish(tw)
        status = self.nav.status()
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        self._publish_viz(now, status, v, omega)
        if now - self._last_map_pub > 1.0:
            self._last_map_pub = now
            self._publish_map()

    def _publish_viz(self, now: float, status: dict, v: float, omega: float):
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link")
        summary = f"{status['state']} lap{status['lap']}  v={v:.2f}m/s  w={omega:+.2f}rad/s"
        if self.avoider.active:
            summary += f"  AVOID {'L' if self.avoider.locked_sign > 0 else 'R'} bias={self.avoider.bias:+.2f}"
        markers = [text_marker(header, "lane_navigator/summary", 0, summary, -1.0, 0.0, 1.4, ORANGE, 0.3)]
        # 1 周目に記憶したコーン地図 (地図座標. 2 周目のライン押し出し・自己位置補正に使うもの)
        map_header = Header(stamp=header.stamp, frame_id=self.map_frame_id)
        for i, (cx, cy) in enumerate(self.cones_avoid.cone_map):
            m = _marker(map_header, "lane_navigator/cone_map", i, Marker.CYLINDER, CONE, (0.3, 0.3, 0.2))
            m.color.a = 0.5
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(cx), float(cy), 0.1
            markers.append(m)
        self.markers_pub.publish(marker_array(header, markers))
        if self.panel_rate <= 0 or now - self._last_panel_t < 1.0 / self.panel_rate:
            return
        self._last_panel_t = now
        tl = self.tl_stop.last_summary
        lines = [
            (f"状態: {STATE_JA.get(status['state'], status['state'])}", WHITE),
            (f"周回: {status['lap']}　指令 v={v:.2f} m/s　ω={omega:+.2f} rad/s", WHITE),
            (f"{status.get('message', '')}", WHITE),
            (f"記録断面 {status['samples']} 点 / 次の間隔 {status['spacing']:.1f}m / 前方曲率 {status['kappa']:+.3f} [1/m]", GRAY),
            (f"自己位置補正 (地図照合) {status['correction']} / 方位ドリフト {status['yaw_drift']:+.4f} rad", GRAY),
        ]
        lines.append((f"コーン: {self.avoider.debug}", YELLOW if self.avoider.active else GRAY))
        ca = self.cones_avoid
        lines.append((f"コーン地図: 1周目に記憶 {len(ca.recorder.cones)} 個 / 確定 {len(ca.cone_map)} 個"
                      + (" (レーシングラインを押し出し済み)" if ca.deflected_points is not None else "")
                      + (f" / 照合 {ca.last_matches} 個" if ca.last_matches else ""), GRAY))
        if tl.get("state") not in (None, "NORMAL"):
            lines.append((f"信号: {tl.get('reason', '')}", YELLOW))
        lines.append((f"信号検出: {tl.get('detect_text', 'なし')}　停止制御: {tl.get('state', '-')}", GRAY))
        img = text_panel(self.jt, "周回マップ + QP 走行の判断", lines, height=300)
        self.panel_pub.publish(cv2_to_imgmsg(img, encoding="bgr8", frame_id="base_link", stamp=header.stamp))

    def _maybe_start_optimization(self):
        cmap = self.nav.pending_course_map()
        if cmap is None or (self._opt_thread is not None and self._opt_thread.is_alive()):
            return
        self._save_map(cmap)

        def work():
            try:
                rl = optimize_raceline(cmap.left, cmap.right, self.params.raceline)
                self.nav.finish_optimization(rl)
                self.get_logger().info(
                    f"レーシングライン生成: {len(rl.points)} ウェイポイント, 1 周 {rl.length:.1f} m, "
                    f"ループ補正 {cmap.closure_error:.2f} m, 方位ドリフト推定 {math.degrees(self.nav.yaw_drift):.1f} deg "
                    f"({'地図に反映' if self.params.lap.yaw_drift_correction else '地図には未反映'})")
            except Exception as e:  # noqa: BLE001
                self.nav.fail(f"レーシングライン生成失敗: {e}")
                self.get_logger().error(f"レーシングライン生成失敗: {e}")

        self._opt_thread = threading.Thread(target=work, daemon=True)
        self._opt_thread.start()

    def _save_map(self, cmap: CourseMap):
        if not self.map_save_path or self._map_saved:
            return
        path = os.path.expanduser(self.map_save_path)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            f.write(cmap.to_json())
        self._map_saved = True
        self.get_logger().info(f"コースマップを保存: {path}")

    def _publish_map(self):
        m = self.nav.course_map
        samples = self.nav.recorder.samples
        left = m.left if m is not None else [s.left for s in samples]
        right = m.right if m is not None else [s.right for s in samples]
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.map_frame_id)
        self.left_pub.publish(self._path(left, header))
        self.right_pub.publish(self._path(right, header))
        if self.nav.raceline is not None:
            # コーン回避で押し出したときは実際に追従しているラインを出す
            dp = self.cones_avoid.deflected_points
            pts = list(dp if dp is not None else self.nav.raceline.points)
            self.raceline_pub.publish(self._path(pts + pts[:1], header))

    @staticmethod
    def _path(points, header) -> Path:
        path = Path(header=header)
        for x, y in points:
            ps = PoseStamped(header=header)
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    # ------------------------------------------------------------------ services
    def _finish_mapping_srv(self, _req, res):
        ok = self.nav.finish_mapping(self.pose, self.s)
        res.success = ok
        res.message = ("1 周目を終了しました. QP 計算中" if ok
                       else f"終了できません (状態 {self.nav.state}, 断面 {len(self.nav.recorder.samples)} 点)")
        return res

    def _reset_srv(self, _req, res):
        self.nav.reset()
        self._map_saved = False
        self.cones_avoid.reset()
        res.success = True
        res.message = "リセットしました (1 周目から)"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = LaneNavigatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
