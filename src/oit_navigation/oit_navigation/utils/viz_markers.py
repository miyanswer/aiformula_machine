#!/usr/bin/env python3
"""
viz_markers.py - 走行判断を RViz2 の 3D 表示で確認するための visualization_msgs/MarkerArray を作る.

座標はすべて base_link (x 前, y 左). RViz2 のテキストマーカーは日本語を描けないので英数字だけにし,
日本語の説明は debug_panel.py のパネル画像に出す.

    six_lane_markers()        仮想6レーンの境界線 (白線=太, 3等分線=細), 現在レーン (緑) / 目標レーン (橙) /
                              コーンで塞がれたレーン (赤) の塗り, 各レーンの確率, 注視点, 判断の要約
    cone_markers()            検出コーン (円柱) と距離
    traffic_light_markers()   検出した信号 (赤/青の球と距離, 距離しか分からないので正面に置く) と
                              停止予定位置の線, 停止制御の状態
    text_marker()             車の上に出す 1 行の要約
"""

import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

LIFETIME = Duration(sec=0, nanosec=500_000_000)  # 更新が止まったら 0.5s で消す (rosbag の一時停止でも残らない)


def rgba(r, g, b, a=1.0) -> ColorRGBA:
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


WHITE, GRAY = rgba(0.95, 0.95, 0.95), rgba(0.55, 0.55, 0.55, 0.8)
GREEN, ORANGE, RED = rgba(0.3, 0.85, 0.3), rgba(1.0, 0.6, 0.15), rgba(0.95, 0.2, 0.2)
YELLOW, CONE = rgba(0.95, 0.9, 0.2), rgba(1.0, 0.45, 0.0)


def _marker(header, ns: str, mid: int, mtype: int, color: ColorRGBA, scale=(1.0, 1.0, 1.0)) -> Marker:
    m = Marker(header=header, ns=ns, id=mid, type=mtype, action=Marker.ADD, color=color, lifetime=LIFETIME)
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
    return m


def delete_all(header) -> Marker:
    return Marker(header=header, action=Marker.DELETEALL)


def text_marker(header, ns: str, mid: int, text: str, x: float, y: float, z: float,
                color: ColorRGBA = WHITE, size: float = 0.35) -> Marker:
    m = _marker(header, ns, mid, Marker.TEXT_VIEW_FACING, color, (0.0, 0.0, size))
    m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
    m.text = ''.join(c if ord(c) < 128 else '?' for c in text)  # RViz2 は ASCII のみ
    return m


def six_lane_markers(header, lines: Optional[Dict], st: Dict, lane_y_fn, n_lanes: int = 6,
                     x_max: float = 10.0) -> List[Marker]:
    out: List[Marker] = []
    cur, tgt = st.get('current_lane'), st.get('target_lane')
    blocked = set(st.get('blocked') or [])
    if lines:
        xs = [0.5 * i for i in range(int(x_max / 0.5) + 1)]
        for F in range(n_lanes + 1):
            main = F % 3 == 0
            m = _marker(header, 'six_lane/boundaries', F, Marker.LINE_STRIP, WHITE if main else GRAY,
                        (0.08 if main else 0.03, 0, 0))
            m.points = [Point(x=x, y=float(lane_y_fn(lines, x, F)), z=0.02) for x in xs]
            out.append(m)
        for k in range(1, n_lanes + 1):
            color = RED if k in blocked else ORANGE if k == tgt else GREEN if k == cur else None
            if color is not None:
                m = _marker(header, 'six_lane/fill', k, Marker.TRIANGLE_LIST, rgba(color.r, color.g, color.b, 0.35))
                for x0, x1 in zip(xs[:-1], xs[1:]):
                    a = Point(x=x0, y=float(lane_y_fn(lines, x0, k - 1)), z=0.01)
                    b = Point(x=x0, y=float(lane_y_fn(lines, x0, k)), z=0.01)
                    c = Point(x=x1, y=float(lane_y_fn(lines, x1, k - 1)), z=0.01)
                    d = Point(x=x1, y=float(lane_y_fn(lines, x1, k)), z=0.01)
                    m.points += [a, b, c, b, d, c]
                out.append(m)
            probs = st.get('probs') or []
            label = f'L{k}' + (f' {probs[k - 1] * 100:.0f}%' if len(probs) >= k else '')
            color = RED if k in blocked else ORANGE if k == tgt else GREEN if k == cur else WHITE
            out.append(text_marker(header, 'six_lane/labels', k, label, 6.0, lane_y_fn(lines, 6.0, k - 0.5), 0.3,
                                   color, 0.3))
    la = st.get('lookahead')
    if la:
        m = _marker(header, 'six_lane/lookahead', 0, Marker.SPHERE, ORANGE, (0.25, 0.25, 0.25))
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(la[0]), float(la[1]), 0.1
        out.append(m)
    if cur:
        summary = (f"L{cur} -> L{tgt}  {st.get('phase', '')}  v={st.get('v', 0):.2f}m/s  "
                   f"conf={st.get('confidence', 0) * 100:.0f}%")
    else:
        summary = f"LINES LOST {st.get('lost_time', 0):.1f}s"
    out.append(text_marker(header, 'six_lane/summary', 0, summary, -1.0, 0.0, 1.4, ORANGE, 0.3))
    return out


def cone_markers(header, cones: Sequence[Tuple[float, float]], conf: Optional[Sequence[float]] = None) -> List[Marker]:
    out: List[Marker] = []
    for i, (x, y) in enumerate(cones):
        m = _marker(header, 'cones', i, Marker.CYLINDER, CONE, (0.3, 0.3, 0.45))
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.225
        out.append(m)
        label = f'{math.hypot(x, y):.1f}m' + (f' ({conf[i]:.2f})' if conf else '')
        out.append(text_marker(header, 'cones/distance', i, label, x, y, 0.75, CONE, 0.25))
    return out


def traffic_light_markers(header, tl: Dict) -> List[Marker]:
    """tl: debug_panel.traffic_light_summary() の戻り値 (TrafficLightStop.status() + 直近の赤/青距離)."""
    out: List[Marker] = []
    for d, color, name, mid in ((tl.get('red_recent'), RED, 'RED', 0), (tl.get('green_recent'), GREEN, 'GREEN', 1)):
        if d is None:
            continue
        m = _marker(header, 'traffic_light/detected', mid, Marker.SPHERE, color, (0.4, 0.4, 0.4))
        m.pose.position.x, m.pose.position.z = float(d), 1.0
        out.append(m)
        out.append(text_marker(header, 'traffic_light/distance', mid, f'{name} {d:.1f}m', d, 0.0, 1.5, color, 0.35))
    state = tl.get('state', 'NORMAL')
    if state in ('APPROACH', 'STOPPED') and tl.get('distance') is not None:
        x_stop = tl['distance'] - tl.get('stop_distance', 7.0)
        m = _marker(header, 'traffic_light/stop_line', 0, Marker.LINE_LIST, YELLOW, (0.12, 0, 0))
        m.points = [Point(x=x_stop, y=2.5, z=0.03), Point(x=x_stop, y=-2.5, z=0.03)]
        out.append(m)
        out.append(text_marker(header, 'traffic_light/stop_line', 1, f'STOP here ({max(x_stop, 0):.1f}m)',
                               x_stop, -2.8, 0.4, YELLOW, 0.3))
    if state != 'NORMAL' or tl.get('red_recent') is not None or tl.get('green_recent') is not None:
        cap = tl.get('v_cap')
        text = f"TL {state}  d={tl['distance']:.1f}m" if tl.get('distance') is not None else f'TL {state}'
        if cap is not None:
            text += f'  vcap={cap:.2f}'
        out.append(text_marker(header, 'traffic_light/state', 0, text, -1.0, 0.0, 1.9, YELLOW, 0.3))
    return out


def marker_array(header, markers: Iterable[Marker]) -> MarkerArray:
    return MarkerArray(markers=[delete_all(header)] + list(markers))
