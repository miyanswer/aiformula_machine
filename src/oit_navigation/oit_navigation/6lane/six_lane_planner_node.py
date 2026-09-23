#!/usr/bin/env python3
"""
six_lane_planner_node.py - 6レーン動的選択走行ノード (地図なし・オドメトリなし).

    入力: aiformula_interfaces/LaneLines (lane_detector)     - 左境界/中央線/右境界 + 検出点群 (base_link)
          can_msgs/Frame (CAN 車輪回転数, id=1809)            - 速度のみ (自己位置は使わない)
          geometry_msgs/PoseArray (任意, cones_topic)         - コーン位置 (base_link). 空文字なら無効
          std_msgs/Float32 赤/青信号までの距離 (traffic_light_distance_node) - 赤なら信号の 5〜10m 手前で停止
    出力: geometry_msgs/Twist -> twist_mux の "mpc" 入力 (/aiformula_control/extremum_seeking_mpc/cmd_vel)
          std_msgs/String (JSON) 判断状態 (/aiformula_control/six_lane_planner/status)
          nav_msgs/Path          目標レーンの中心線 (base_link, /aiformula_visualization/six_lane_planner/target_path)
          visualization_msgs/MarkerArray 仮想6レーン・現在/目標レーン・確率・注視点 (base_link, .../six_lane_planner/markers)
          sensor_msgs/Image      判断パネル (俯瞰図 + 確率バー + 日本語の判断理由, .../six_lane_planner/panel)
          (status JSON にも日本語の判断理由 'explain' を入れる. 走行後に rosbag を再生して RViz で確認する用)

毎フレーム, 3本の白線から仮想6レーン (左白線〜中央線を3等分 = レーン1〜3, 中央線〜右白線 = レーン4〜6) を作り,
前方の曲率 (近/中/遠) と速度から MLP (six_lane_policy.json) が行くべきレーンを選ぶ (左回りコースでの
アウト・イン・アウト: 直線=レーン6, カーブ中=イン側, 脱出=アウト側). lane_navigator (周回マップ + QP) の
代わりに起動する (同じ cmd_vel トピックに出すので同時起動しないこと).

アルゴリズム本体は six_lane_core.py (web_simulator/js/six_lane_planner.js と同一).
"""

import json
import math
import os
import struct
import threading
from typing import Dict, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from can_msgs.msg import Frame
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Header, String
from visualization_msgs.msg import MarkerArray

from aiformula_interfaces.msg import LaneLine, LaneLines
from oit_navigation.utils.traffic_light_stop_ros import TrafficLightStopRos

from oit_navigation.lane_nav.cone_avoidance import ReactiveAvoider
from oit_navigation.utils.debug_panel import JapaneseText, six_lane_panel
from oit_navigation.utils.image_util import cv2_to_imgmsg
from oit_navigation.utils.viz_markers import cone_markers, marker_array, six_lane_markers
from .six_lane_core import LOST, ROLES, LanePolicyNet, LineObs, SixLaneParams, SixLanePlanner, explain_ja, lane_y

RPM_ID = 1809  # odometry_publisher/wheel.hpp と同じ


def lane_line_to_obs(msg: LaneLine) -> Optional[LineObs]:
    if not msg.valid:
        return None
    detected = bool(msg.detected)
    return LineObs(c=(msg.c0, msg.c1, msg.c2), x_min=msg.x_min, x_max=msg.x_max, detected=detected,
                   px=list(msg.points_x) if detected else None, py=list(msg.points_y) if detected else None)


def _round(v, nd=3):
    return round(float(v), nd) if isinstance(v, (int, float)) else v


def status_json(st: Dict) -> Dict:
    """web_simulator の sixLaneStatusJson() と同じキー."""
    return {
        'phase': st.get('phase'), 'current_lane': st.get('current_lane'), 'target_lane': st.get('target_lane'),
        'pending_lane': st.get('pending_lane'), 'pending_count': st.get('pending_count', 0),
        'sign': st.get('sign', 0), 'teacher_target': _round(st.get('teacher_target')),
        'intensity': _round(st.get('intensity')), 'F': _round(st.get('F')), 'F_meas': _round(st.get('F_meas')),
        'lateral_rejected': bool(st.get('lateral_rejected', False)),
        'kappas': [_round(k, 4) for k in st.get('kappas', [])], 'confidence': _round(st.get('confidence')),
        'nn_probs': [_round(q) for q in st.get('nn_probs', [])], 'probs': [_round(q) for q in st.get('probs', [])],
        'blocked': st.get('blocked', []), 'v': _round(st.get('v')), 'omega': _round(st.get('omega')),
        'lost_time': _round(st.get('lost_time', 0.0)),
        'explain': st.get('explain', []),
        'avoid': st.get('avoid', ''),
    }


class SixLanePlannerNode(Node):
    def __init__(self):
        super().__init__('six_lane_planner')
        params = self._load_params()
        net = LanePolicyNet.load(self.policy_path)
        self.planner = SixLanePlanner(net, params)

        self.v = 0.0
        self._lock = threading.Lock()
        self._pending: Optional[Dict[str, Optional[LineObs]]] = None
        self._reanchored = False
        self._last_lines_t: Optional[float] = None
        self._last_step_t: Optional[float] = None
        self._cones = []
        self._cones_t = -1e9
        self._last_raw_cmd = (0.0, 0.0)   # planner の生の (v, omega). 信号の速度上限を掛ける前
        self._last_cmd_t: Optional[float] = None

        self.create_subscription(LaneLines, self.lane_lines_topic, self._lines_cb, 5)
        self.create_subscription(Frame, self.can_topic, self._can_cb, 50)
        if self.cones_topic:
            self.create_subscription(PoseArray, self.cones_topic, self._cones_cb, 5)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 1)
        self.status_pub = self.create_publisher(String, self.status_topic, 1)
        self.path_pub = self.create_publisher(Path, self.target_path_topic, 1)
        self.reseed_pub = self.create_publisher(Float64, self.lane_reseed_topic, 1)
        self.markers_pub = self.create_publisher(MarkerArray, self.markers_topic, 1)
        self.panel_pub = self.create_publisher(Image, self.panel_topic, 1)
        self.jt = JapaneseText(self.panel_font_path)
        if not self.jt.available:
            self.get_logger().warning('判断パネル用の日本語フォントがありません (sudo apt install fonts-noto-cjk). 日本語は ? になります')
        self._last_panel_t = -1e9
        self._last_lines: Optional[Dict[str, Optional[LineObs]]] = None
        # 赤信号停止: 最終 cmd_vel に速度上限を掛ける (lane_navigator と共通, utils/traffic_light_stop.py)
        self.tl_stop = TrafficLightStopRos(self)
        # コーンの反応的回避: レーン除外 (planner) の上に重ねる最終安全層 (シミュレータの stepSixLane と同じ)
        self.avoider = ReactiveAvoider(enabled=bool(self.declare_parameter('cone_avoidance.reactive', True).value))
        self.create_timer(1.0 / self.control_rate, self._control)
        self.get_logger().info(
            f'six_lane_planner: lines={self.lane_lines_topic} CAN={self.can_topic} '
            f'cones={self.cones_topic or "(なし)"} -> cmd_vel={self.cmd_vel_topic} (policy {self.policy_path})')

    # ------------------------------------------------------------------ params
    def _load_params(self) -> SixLaneParams:
        d = self.declare_parameter
        self.lane_lines_topic = d('lane_lines_topic', '/aiformula_perception/lane_detector/lane_lines').value
        self.can_topic = d('can_topic', '/aiformula_sensing/vehicle_info').value
        self.cones_topic = str(d('cones_topic', '').value)
        self.cmd_vel_topic = d('cmd_vel_topic', '/aiformula_control/extremum_seeking_mpc/cmd_vel').value
        self.status_topic = d('status_topic', '/aiformula_control/six_lane_planner/status').value
        self.target_path_topic = d('target_path_topic', '/aiformula_visualization/six_lane_planner/target_path').value
        # 白線の役割取り違えを検出したら, 追跡済みの横位置 F を lane_detector に送って線の並びを置き直させる
        self.lane_reseed_topic = d('lane_reseed_topic', '/aiformula_control/six_lane_planner/lane_reseed').value
        # RViz 用の可視化 (走行後に rosbag を再生して判断を確認する)
        self.markers_topic = d('markers_topic', '/aiformula_visualization/six_lane_planner/markers').value
        self.panel_topic = d('panel_topic', '/aiformula_visualization/six_lane_planner/panel').value
        self.panel_rate = float(d('panel_rate', 5.0).value)     # [Hz] 判断パネル画像 (0 で出さない)
        self.panel_font_path = str(d('panel_font_path', '').value)  # 空なら Noto CJK などを自動で探す
        self.frame_id = d('frame_id', 'base_link').value
        self.control_rate = float(d('control_rate', 20.0).value)
        self.lines_timeout = float(d('lines_timeout', 0.3).value)  # これより新しい白線が来なければ「観測なし」で1周期進める
        self.cones_timeout = float(d('cones_timeout', 0.5).value)
        self.wheel_diameter = float(d('wheel.diameter', 0.254).value)
        default_policy = os.path.join(get_package_share_directory('oit_navigation'), '6lane', 'six_lane_policy.json')
        self.policy_path = os.path.expanduser(str(d('policy_path', default_policy).value))

        p = SixLaneParams()
        for name in p.__dataclass_fields__:
            default = getattr(p, name)
            if isinstance(default, tuple):
                setattr(p, name, tuple(float(v) for v in d(name, [float(v) for v in default]).value))
            elif isinstance(default, (bool, int, float)):
                setattr(p, name, type(default)(d(name, default).value))
        return p

    # ------------------------------------------------------------------ io
    def _lines_cb(self, msg: LaneLines):
        lines = {r: lane_line_to_obs(getattr(msg, r)) for r in ROLES}
        with self._lock:
            # 付け直しフラグは次の制御周期まで取りこぼさないよう OR で溜める
            self._reanchored = self._reanchored or bool(getattr(msg, 'reanchored', False))
            self._pending = lines

    def _can_cb(self, msg: Frame):
        if msg.id != RPM_ID or len(msg.data) < 8:
            return
        rpm_right, rpm_left = struct.unpack('<ii', bytes(msg.data[:8]))
        self.v = 0.5 * (rpm_left + rpm_right) / 60.0 * math.pi * self.wheel_diameter

    def _cones_cb(self, msg: PoseArray):
        with self._lock:
            self._cones = [(ps.position.x, ps.position.y) for ps in msg.poses]
            self._cones_t = self._now()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ control
    def _control(self):
        now = self._now()
        with self._lock:
            lines, self._pending = self._pending, None
            reanchored, self._reanchored = self._reanchored, False
            cones = list(self._cones) if now - self._cones_t < self.cones_timeout else []
        if lines is not None:
            self._last_lines_t = now
        elif self._last_lines_t is not None and now - self._last_lines_t < self.lines_timeout:
            # 次の白線フレーム待ち: 直前の指令を維持 (信号の速度上限は毎周期掛け直す)
            self._publish_cmd(now, self._last_raw_cmd, cones)
            return
        dt = 0.0 if self._last_step_t is None else min(now - self._last_step_t, 0.5)
        self._last_step_t = now
        usable = lines if lines is not None and all(lines[r] is not None for r in ROLES) else None
        st = self.planner.step(dt, usable, self.v, cones, reanchored)
        st['explain'] = explain_ja(st, self.planner.p)
        if usable is not None:
            self._last_lines = usable

        self._last_raw_cmd = (float(st['v']), float(st['omega']))
        self._publish_cmd(now, self._last_raw_cmd, cones)
        st['avoid'] = self.avoider.debug
        self.status_pub.publish(String(data=json.dumps(status_json(st), ensure_ascii=False)))
        if st.get('lateral_rejected'):
            self.reseed_pub.publish(Float64(data=float(st['F'])))
        if st['phase'] != LOST and usable is not None:
            self._publish_target_path(usable, st['target_lane'])
        if st['phase'] == LOST:
            self.get_logger().warning('白線を見失っています', throttle_duration_sec=2.0)
        self._publish_viz(now, usable, st, cones)

    def _publish_viz(self, now: float, lines, st: Dict, cones):
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame_id)
        markers = six_lane_markers(header, lines, st, lane_y)
        # 回避に使ったコーン (cones_topic) も同じ表示に出す (検出器側の表示とは別に, 判断に使った入力として)
        markers += [m for m in cone_markers(header, cones) if m.ns == 'cones']
        for m in markers:
            if m.ns == 'cones':
                m.ns = 'six_lane/cones_used'
        self.markers_pub.publish(marker_array(header, markers))
        if self.panel_rate <= 0 or now - self._last_panel_t < 1.0 / self.panel_rate:
            return
        self._last_panel_t = now
        tl = self.tl_stop.last_summary
        img = six_lane_panel(self.jt, lines, st, st.get('explain', []) + [f'コーン回避: {self.avoider.debug}'],
                             lane_y, cones=cones, tl=tl)
        self.panel_pub.publish(cv2_to_imgmsg(img, encoding='bgr8', frame_id=self.frame_id, stamp=header.stamp))

    def _publish_cmd(self, now: float, raw, cones):
        dt = 0.0 if self._last_cmd_t is None else min(now - self._last_cmd_t, 0.5)
        self._last_cmd_t = now
        v, omega = self.avoider.step(now, dt, raw[0], raw[1], cones)
        v, omega = self.tl_stop.apply(now, dt, self.v, v, omega)
        tw = Twist()
        tw.linear.x = float(v)
        tw.angular.z = float(omega)
        self.cmd_pub.publish(tw)

    def _publish_target_path(self, lines, target_lane: int):
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame_id)
        path = Path(header=header)
        for i in range(21):
            x = 0.5 * i
            ps = PoseStamped(header=header)
            ps.pose.position.x = x
            ps.pose.position.y = float(lane_y(lines, x, target_lane - 0.5))
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = SixLanePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
