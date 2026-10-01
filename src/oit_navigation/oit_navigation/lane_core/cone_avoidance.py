"""
コーン回避 (ROS 非依存). web_simulator/js/cone_avoidance.js の移植 (定数・計算は同一. 変えたら両方直すこと).

    ReactiveAvoider  毎制御周期: 走行ノード (six_lane_planner) の指令に, 前方のコーン群 (半径 keep_out_radius の
                     禁止円の和集合) の外周円弧をなぞって抜ける操舵バイアスを足す. 近いときは減速する.
"""

import math
from typing import List, Sequence, Tuple

KEEP_OUT_RADIUS = 1.0          # [m] 各コーン中心からの侵入禁止半径
REACT_CLEARANCE = KEEP_OUT_RADIUS
REACT_LOOKAHEAD_X = 7.0
REACT_MIN_X = 0.2
REACT_MAX_OMEGA_BIAS = 0.85
REACT_SLOW_X = 1.8
REACT_SLOW_V = 0.6
CLUSTER_LINK_DISTANCE = KEEP_OUT_RADIUS * 2
THREAT_HOLD_TIME = 0.8         # [s]

Cone = Tuple[float, float]


def rate_limit(prev: float, target: float, max_rate: float, dt: float) -> float:
    step = max_rate * max(dt, 0.0)
    return prev + max(-step, min(step, target - prev))


def cluster_cones(cones: Sequence[Cone]) -> List[dict]:
    """禁止円が接するコーンを連結成分ごとのクラスタにまとめる (js clusterConeDetections)."""
    remaining = list(range(len(cones)))
    clusters = []
    while remaining:
        seed = remaining.pop(0)
        idx = [seed]
        cursor = 0
        while cursor < len(idx):
            ax, ay = cones[idx[cursor]]
            for i in list(remaining):
                bx, by = cones[i]
                if math.hypot(ax - bx, ay - by) <= CLUSTER_LINK_DISTANCE:
                    remaining.remove(i)
                    idx.append(i)
            cursor += 1
        members = [cones[i] for i in idx]
        clusters.append({'cones': members, 'min_x': min(c[0] for c in members),
                         'center_y': sum(c[1] for c in members) / len(members)})
    return clusters


class ReactiveAvoider:
    """js reactiveAvoid() と同じ. JS はモジュール変数で持つ状態 (回避方向のロック・最後に脅威を見た時刻・
    前回のバイアス) をインスタンスに持つ."""

    def __init__(self, max_rate: float = 4.0, enabled: bool = True):
        self.max_rate = max_rate
        self.enabled = enabled
        self.reset()

    def reset(self):
        self.locked_sign = 0
        self.last_threat_time = 0.0
        self.bias = 0.0
        self.debug = '回避待機'
        self.active = False
        self.target_offset = 0.0

    def step(self, now: float, dt: float, v_cmd: float, omega_cmd: float,
             cones: Sequence[Cone], scale: float = 1.0) -> Tuple[float, float]:
        """scale: 現在の車速に応じた自動補正倍率 (six_lane_core.effective_control(p, v)['react_scale']). 減速を始める距離と
        回避を計算する前方距離を, 車速に比例して遠くする (速いほど手前から避け始める)."""
        if not self.enabled:
            self.active = False
            return v_cmd, omega_cmd
        valid = [c for c in cones if REACT_MIN_X <= c[0] <= REACT_LOOKAHEAD_X and abs(c[1]) < 3.0]
        clusters = cluster_cones(valid)
        threats = sorted((cl for cl in clusters if any(abs(c[1]) < REACT_CLEARANCE for c in cl['cones'])),
                         key=lambda cl: cl['min_x'])
        threat = threats[0] if threats else None
        if threat is not None:
            self.last_threat_time = now
        if threat is None or now - self.last_threat_time > THREAT_HOLD_TIME:
            self.locked_sign = 0
            self.bias = rate_limit(self.bias, 0.0, self.max_rate, dt)
            self.active = False
            self.debug = f'回避待機: 有効コーン {len(valid)} 本 / クラスタ {len(clusters)} 個'
            return v_cmd, omega_cmd + self.bias

        # コース中心に対して左 (y>0) のクラスタは右へ, 右のクラスタは左へ抜ける (通過中はロック)
        desired = -1 if threat['center_y'] > 0 else 1
        if self.locked_sign == 0 or self.locked_sign != desired:
            self.locked_sign = desired
        min_x = threat['min_x']
        close_slow = min_x < REACT_SLOW_X * scale
        lookahead_x = max(0.9, min(min_x, 3.5 * scale))
        required = 0.0
        for cx, cy in threat['cones']:
            dx = lookahead_x - cx
            if abs(dx) < REACT_CLEARANCE:
                arc = math.sqrt(REACT_CLEARANCE * REACT_CLEARANCE - dx * dx)
                if self.locked_sign > 0:
                    required = max(required, cy + arc)
                else:
                    required = min(required, cy - arc)
            elif cx > lookahead_x:
                direct = cy + REACT_CLEARANCE if self.locked_sign > 0 else cy - REACT_CLEARANCE
                if self.locked_sign > 0 and direct > required:
                    required = direct
                if self.locked_sign < 0 and direct < required:
                    required = direct
        dist_sq = lookahead_x * lookahead_x + required * required
        curvature = 2.0 * required / max(dist_sq, 1.0)
        v = min(v_cmd, REACT_SLOW_V) if close_slow else v_cmd
        target = curvature * max(v, 0.9) * 1.3
        target = max(-REACT_MAX_OMEGA_BIAS, min(REACT_MAX_OMEGA_BIAS, target))
        self.bias = rate_limit(self.bias, target, self.max_rate, dt)
        self.active = True
        self.target_offset = required
        side = '左' if self.locked_sign > 0 else '右'
        self.debug = (f"回避中: {len(threat['cones'])} 本のクラスタを{side}へ回避 | 禁止半径 {KEEP_OUT_RADIUS:.1f} m | "
                      f"横目標 {required:.2f} m | 操舵補正 {self.bias:.2f} rad/s")
        return v, omega_cmd + self.bias
