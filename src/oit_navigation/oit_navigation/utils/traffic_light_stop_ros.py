#!/usr/bin/env python3
"""
traffic_light_stop_ros.py - TrafficLightStop (traffic_light_stop.py) を走行ノードに組み込む ROS 側の薄い層.

lane_navigator / six_lane_planner の両方が同じように使う:

    self.tl_stop = TrafficLightStopRos(self)                 # パラメータ宣言 + 購読 + status 配信
    v, omega = self.tl_stop.apply(now, dt, v_meas, v, omega)  # 毎制御周期, cmd_vel を出す直前

    入力: <traffic_light_stop.red_distance_topic>   std_msgs/Float32  最も近い赤信号までの距離 [m]
          <traffic_light_stop.green_distance_topic> std_msgs/Float32  最も近い青信号までの距離 [m]
          (traffic_light_distance_node が検出したフレームだけ出す. 同じ画像フレームの赤/青は
           同時刻に届くので, 受信時刻が近いものを 1 フレームにまとめてから TrafficLightStop に渡す)
    出力: <traffic_light_stop.status_topic>         std_msgs/String (JSON)
          <traffic_light_stop.markers_topic>        visualization_msgs/MarkerArray (base_link, RViz 用:
                                                    検出した赤/青信号と距離, 停止予定位置の線, 状態)
"""

import json
import threading
from dataclasses import fields
from typing import Optional, Tuple

from std_msgs.msg import Float32, Header, String
from visualization_msgs.msg import MarkerArray

from .debug_panel import traffic_light_summary
from .traffic_light_stop import TrafficLightStop, TrafficLightStopParams
from .viz_markers import marker_array, traffic_light_markers

PREFIX = 'traffic_light_stop.'


class TrafficLightStopRos:
    def __init__(self, node):
        self.node = node
        d = node.declare_parameter
        p = TrafficLightStopParams()
        for f in fields(p):
            default = getattr(p, f.name)
            setattr(p, f.name, type(default)(d(PREFIX + f.name, default).value))
        red_topic = d(PREFIX + 'red_distance_topic', '/aiformula_perception/traffic_light/red_distance').value
        green_topic = d(PREFIX + 'green_distance_topic', '/aiformula_perception/traffic_light/green_distance').value
        status_topic = d(PREFIX + 'status_topic', '/aiformula_control/traffic_light_stop/status').value
        markers_topic = d(PREFIX + 'markers_topic', '/aiformula_visualization/traffic_light_stop/markers').value
        self.frame_id = d(PREFIX + 'frame_id', 'base_link').value
        self.recent_time = float(d(PREFIX + 'recent_time', 0.5).value)   # [s] これ以内に見えた信号を「見えている」と表示

        self.stop = TrafficLightStop(p)
        self._lock = threading.Lock()
        self._red: Optional[Tuple[float, float]] = None     # (受信時刻, 距離)
        self._green: Optional[Tuple[float, float]] = None
        node.create_subscription(Float32, red_topic, lambda m: self._cb(m, 'red'), 10)
        node.create_subscription(Float32, green_topic, lambda m: self._cb(m, 'green'), 10)
        self.status_pub = node.create_publisher(String, status_topic, 1)
        self.markers_pub = node.create_publisher(MarkerArray, markers_topic, 1)
        self._last_state = self.stop.state
        self.last_summary = traffic_light_summary(self.stop.status(), None, None)
        node.get_logger().info(
            f'traffic_light_stop: {"有効" if p.enabled else "無効"} red={red_topic} green={green_topic} '
            f'-> 信号機の {p.stop_distance:.1f}m 手前で停止 (許容 {p.stop_distance_min:.0f}〜{p.stop_distance_max:.0f}m)')

    def summary(self, now: float):
        """status + 直近 recent_time 秒以内に見えた赤/青の距離 (パネル・マーカー表示用)."""
        s = self.stop
        red = s.last_red_distance if s.last_red_t is not None and now - s.last_red_t <= self.recent_time else None
        green = (s.last_green_distance if s.last_green_t is not None and now - s.last_green_t <= self.recent_time
                 else None)
        return traffic_light_summary(s.status(), red, green)

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _cb(self, msg: Float32, color: str):
        with self._lock:
            setattr(self, '_' + color, (self._now(), float(msg.data)))

    def apply(self, now: float, dt: float, v_meas: float, v: float, omega: float) -> Tuple[float, float]:
        with self._lock:
            red, green, self._red, self._green = self._red, self._green, None, None
        if red is not None or green is not None:
            t = max(x[0] for x in (red, green) if x is not None)
            self.stop.observe(t, red[1] if red else None, green[1] if green else None)
        v_out, omega_out = self.stop.apply(now, dt, v_meas, v, omega)
        st = self.stop.status()
        self.status_pub.publish(String(data=json.dumps(st, ensure_ascii=False)))
        self.last_summary = self.summary(now)
        header = Header(stamp=self.node.get_clock().now().to_msg(), frame_id=self.frame_id)
        self.markers_pub.publish(marker_array(header, traffic_light_markers(header, self.last_summary)))
        if st['state'] != self._last_state:
            self.node.get_logger().info(f'[信号] {self._last_state} -> {st["state"]}: {st["reason"]}')
            self._last_state = st['state']
        return v_out, omega_out
