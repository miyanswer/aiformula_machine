"""
コーン回避 (ROS 非依存). web_simulator/js/cone_avoidance.js の移植 (定数・計算は同一. 変えたら両方直すこと).

    ReactiveAvoider           毎制御周期: 走行ノードの指令に, 前方のコーン群 (半径 keep_out_radius の禁止円の和集合)
                              の外周円弧をなぞって抜ける操舵バイアスを足す. 近いときは減速する.
                              周回マップ + QP (lane_navigator) と 6レーン (six_lane_planner) の両方の最終安全層.
    ConeRecorder              1 周目 (MAPPING): 検出したコーンを記憶し, 周回完了時にコースマップと同じ補正
                              (方位ドリフト・ループ閉じ込み) を掛けて地図座標のコーン地図にする.
    deflect_raceline_around_cones  2 周目以降: QP のレーシングラインを記憶したコーンから禁止半径の外へ押し出す.
    cone_landmark_correction  2 周目以降: 見えているコーンを地図上のコーンと照合し, 自己位置 (x, y) を補正する.
"""

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .boundary_recorder import BoundarySample, vehicle_to_world
from .course_map import CourseMap, LapDetectorParams, corrected_poses, yaw_drift_applied
from .path_tracker import densify_closed, rate_limit

KEEP_OUT_RADIUS = 1.0          # [m] 各コーン中心からの侵入禁止半径
REACT_CLEARANCE = KEEP_OUT_RADIUS
REACT_LOOKAHEAD_X = 7.0
REACT_MIN_X = 0.2
REACT_MAX_OMEGA_BIAS = 0.85
REACT_SLOW_X = 1.8
REACT_SLOW_V = 0.6
CLUSTER_LINK_DISTANCE = KEEP_OUT_RADIUS * 2
THREAT_HOLD_TIME = 0.8         # [s]
# 2 周目のラインはコーン中心から 1.3m 離す (反応的回避の禁止半径 1.0m より広い). 地図の誤差 (~0.3m) と
# 追従のショートカット (~0.2m) が重なっても, 車体 (半幅 0.35m) がコーン (半径 0.15m) に触れない余裕を持たせる
DEFLECT_CLEARANCE = 1.3
DEFLECT_STEP = 0.2             # [m] 押し出す前に細かくする間隔 (RacelineFollower と同じ)
DEFLECT_WINDOW = 4.5           # [m] コーンの前後この範囲に押し出しを配る (広いほど追従でショートカットしにくい)
DEFLECT_ITERATIONS = 4
DEFLECT_EDGE_MARGIN = 0.5      # [m] 押し出したラインをコース境界からこれだけ内側に保つ (車体半幅 0.4 + 0.1)

Cone = Tuple[float, float]


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
             cones: Sequence[Cone]) -> Tuple[float, float]:
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
        close_slow = min_x < REACT_SLOW_X
        lookahead_x = max(0.9, min(min_x, 3.5))
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


class ConeRecorder:
    """1 周目のコーン記憶 (js ConeRecorder). 同じコーンは gate_radius 以内で名寄せし, 最も近くで見た観測を残す."""

    def __init__(self, gate_radius: float = 0.6):
        self.gate_radius = gate_radius
        self.cones: List[dict] = []

    def reset(self):
        self.cones = []

    def update(self, s: float, pose, detections: Sequence[Cone]):
        for dx, dy in detections:
            wx, wy = vehicle_to_world(pose, dx, dy)   # 名寄せ用の未補正ラフ座標
            hit = next((c for c in self.cones if math.hypot(c['rough_x'] - wx, c['rough_y'] - wy) < self.gate_radius),
                       None)
            if hit is None:
                self.cones.append({'rough_x': wx, 'rough_y': wy, 's': s, 'local_x': dx, 'local_y': dy})
            elif dx < hit['local_x']:
                hit.update(rough_x=wx, rough_y=wy, s=s, local_x=dx, local_y=dy)

    def finalize(self, samples: List[BoundarySample], course_map: Optional[CourseMap],
                 lap: LapDetectorParams, yaw_drift: float) -> List[Cone]:
        """周回完了時に 1 回呼ぶ. 境界点と同じ補正 (方位ドリフト補正は地図が掛けたときだけ + ループ閉じ込み) で
        記録時の姿勢から再投影し, コースマップ (= レーシングライン) と同じ座標のコーン地図を返す."""
        if not samples or not self.cones:
            return []
        if yaw_drift_applied(lap, yaw_drift):
            poses = corrected_poses(samples, yaw_drift)
        else:
            poses = [np.asarray(sm.pose, float) for sm in samples]
        ss = np.array([sm.s for sm in samples])
        out = []
        for c in self.cones:
            bi = int(np.argmin(np.abs(ss - c['s'])))
            x, y = vehicle_to_world(poses[bi], c['local_x'], c['local_y'])
            if course_map is not None and course_map.closure_offsets is not None and len(course_map.s):
                k = int(np.argmin(np.abs(course_map.s - c['s'])))
                x += float(course_map.closure_offsets[k][0])
                y += float(course_map.closure_offsets[k][1])
            out.append((float(x), float(y)))
        return out


def deflect_raceline_around_cones(points: np.ndarray, speeds: np.ndarray, cones: Sequence[Cone],
                                  left: Optional[np.ndarray] = None, right: Optional[np.ndarray] = None,
                                  step: float = DEFLECT_STEP, window: float = DEFLECT_WINDOW,
                                  iterations: int = DEFLECT_ITERATIONS) -> Tuple[np.ndarray, np.ndarray]:
    """QP のレーシングライン (閉曲線) を, 記憶したコーンから DEFLECT_CLEARANCE 離れるよう滑らかに押し出す.

    QP のウェイポイントは直線部で数 m おきしかないので, 点だけ押すと間をつなぐスプラインがコーンの脇を通る.
    そこで先に step [m] 間隔に細かくしてから, コーンに最も近い点を横方向に押し, 前後 window [m] に同じ押し出しを
    raised cosine の重みで配る (山形のこぶ). 押し残しがあれば繰り返す.
    left/right (コースマップの左右境界) があれば, 押し出す側をコースの幅で決める: コーンの左右それぞれ
    「コーンから DEFLECT_CLEARANCE, 境界から DEFLECT_EDGE_MARGIN 内側」に入れる側のうち今のラインに近い側へ.
    どちらにも入れなければ広い側の限界まで (残りは反応的回避と減速で補う). 境界が無ければコーンから放射状に押す.
    戻り値は細かくした点列と速度 (RacelineFollower にそのまま渡せる).
    """
    out, v = densify_closed(np.asarray(points, float), np.asarray(speeds, float), step)
    n = len(out)
    half = max(1, int(math.floor(window / step + 0.5)))   # JS の Math.round と同じ (round() は偶数丸め)
    ks = np.arange(-half, half + 1)
    weights = 0.5 * (1.0 + np.cos(np.pi * ks / (half + 1)))
    have_bounds = left is not None and right is not None and len(left) >= 2
    if have_bounds:
        left, right = np.asarray(left, float), np.asarray(right, float)
        mids = 0.5 * (left + right)
    for _ in range(iterations):
        moved = False
        for cx, cy in cones:
            c = np.array([cx, cy])
            d = np.hypot(out[:, 0] - cx, out[:, 1] - cy)
            i = int(np.argmin(d))
            if d[i] >= DEFLECT_CLEARANCE - 1e-3:
                continue
            if have_bounds:
                j = int(np.argmin(np.hypot(mids[:, 0] - cx, mids[:, 1] - cy)))
                across = left[j] - right[j]
                width = float(np.linalg.norm(across))
                nrm = across / max(width, 1e-9)                   # 右境界 -> 左境界 の向き
                c_lat = float((c - right[j]) @ nrm)               # 右境界からの横位置
                p_lat = float((out[i] - right[j]) @ nrm)
                lo, hi = DEFLECT_EDGE_MARGIN, width - DEFLECT_EDGE_MARGIN
                sides = [t for t in (c_lat + DEFLECT_CLEARANCE, c_lat - DEFLECT_CLEARANCE) if lo <= t <= hi]
                if sides:
                    target = min(sides, key=lambda t: abs(t - p_lat))
                else:   # どちらにも入れない: 広い側の限界まで
                    target = hi if (hi - c_lat) >= (c_lat - lo) else lo
                push = (target - p_lat) * nrm
                if abs(target - p_lat) < 1e-3:
                    continue
            else:
                u = (out[i] - c) / max(d[i], 1e-6)
                push = (DEFLECT_CLEARANCE - d[i]) * u
            idx = (i + ks) % n
            out[idx] += weights[:, None] * push[None, :]
            moved = True
        if not moved:
            break
    return out, v


def cone_landmark_correction(navigator, pose, detections: Sequence[Cone], cone_map: Sequence[Cone],
                             gate: float = 1.0) -> int:
    """見えているコーンを地図上のコーンと照合し, 平均のずれを navigator の自己位置補正に足す (位置のみ).
    戻り値: 照合できたコーンの数."""
    if not cone_map or not detections:
        return 0
    mp = navigator.map_pose(pose)
    sx = sy = 0.0
    count = 0
    for dx, dy in detections:
        wx, wy = vehicle_to_world(mp, dx, dy)
        best, best_d = None, math.inf
        for c in cone_map:
            dd = math.hypot(c[0] - wx, c[1] - wy)
            if dd < best_d:
                best, best_d = c, dd
        if best is not None and best_d < gate:
            sx += best[0] - wx
            sy += best[1] - wy
            count += 1
    if count:
        navigator.apply_external_correction(sx / count, sy / count, 0.0)
    return count


class NavigatorConeAvoidance:
    """周回マップ + QP 走行 (LaneNavigator) のコーン回避の段取り. lane_navigator ノードと
    オフラインのテストが同じものを使う (シミュレータの stepNavigator() と同じ順序).

        observe()  コーン検出が届くたび: 1 周目 (MAPPING) なら記憶する
        step()     制御周期ごと: RACING に入った直後にコーン地図を確定してレーシングラインを押し出し,
                   2 周目以降は新しい検出ごとにランドマーク照合で自己位置を補正, 最後に反応的回避を指令に足す
    """

    def __init__(self, reactive: bool = True, deflect: bool = True, landmark: bool = True, gate: float = 1.0):
        self.avoider = ReactiveAvoider(enabled=reactive)
        self.recorder = ConeRecorder()
        self.deflect = deflect
        self.landmark = landmark
        self.gate = gate
        self.reset()

    def reset(self):
        self.avoider.reset()
        self.recorder.reset()
        self.cone_map: List[Cone] = []
        self.deflected_points: Optional[np.ndarray] = None
        self._prev_state = None
        self.last_matches = 0

    def observe(self, nav, s: float, pose, cones: Sequence[Cone]):
        if nav.state == 'MAPPING' and pose is not None:
            self.recorder.update(s, pose, cones)

    def on_racing_started(self, nav, lap: LapDetectorParams, tracker_params):
        """RACING に入った直後に 1 回. cone_map が既にあれば (保存済みマップから再開) それを使う.
        戻り値: 新しくコーン地図を確定したら True (保存用)."""
        from .path_tracker import RacelineFollower
        created = False
        if not self.cone_map:
            self.cone_map = self.recorder.finalize(nav.recorder.samples, nav.course_map, lap, nav.yaw_drift)
            created = bool(self.cone_map)
        if self.cone_map and self.deflect and nav.raceline is not None:
            m = nav.course_map
            pts, sp = deflect_raceline_around_cones(nav.raceline.points, nav.raceline.speed, self.cone_map,
                                                    m.left if m is not None else None, m.right if m is not None else None)
            nav.follower = RacelineFollower(pts, sp, tracker_params)
            self.deflected_points = pts
        return created

    def step(self, now: float, dt: float, nav, pose, v_cmd: float, omega_cmd: float, cones: Sequence[Cone],
             cones_new: bool, lap: LapDetectorParams, tracker_params) -> Tuple[float, float, bool]:
        """戻り値: (v, omega, コーン地図を新しく確定したか)."""
        created = False
        if nav.state == 'RACING' and self._prev_state != 'RACING' and nav.raceline is not None:
            created = self.on_racing_started(nav, lap, tracker_params)
        self._prev_state = nav.state
        self.last_matches = 0
        if nav.state == 'RACING' and self.landmark and cones_new and self.cone_map:
            self.last_matches = cone_landmark_correction(nav, pose, cones, self.cone_map, self.gate)
        v, omega = self.avoider.step(now, dt, v_cmd, omega_cmd, cones)
        return v, omega, created
