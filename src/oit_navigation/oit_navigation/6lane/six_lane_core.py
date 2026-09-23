"""six_lane_core.py - 6レーン動的選択走行のROS非依存コア.

地図なし・オドメトリなし. 毎フレームのカメラ白線 (左/中央/右, base_link) だけから
    1. 仮想6レーン (左白線〜中央線を3等分 = レーン1〜3, 中央線〜右白線を3等分 = レーン4〜6)
    2. 前方の曲率プロファイル (近 3m / 中 6m / 遠 9.5m)
    3. 現在の横位置 (レーン座標 F: 左白線=0, 中央線=3, 右白線=6) と現在レーン
を求め, 小さなニューラルネット (MLP) が「速度とコース形状からどのレーンへ行くか」を
6レーンの確率として出す. その後コーンで塞がれたレーンを除外し, ヒステリシスで目標レーンを
確定, 目標レーンへ進入角制限つき Pure Pursuit で追従する.

左回りコース想定: 左カーブのアウト = レーン6, イン = レーン1. 直線ではアウト側 (home_lane=6).

web_simulator/js/six_lane_planner.js はこのファイルの移植 (同じパラメータ名・同じ計算).
NN の重みは six_lane_policy.json (train_policy.py で生成) を両者で共有する.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

N_LANES = 6
ROLES = ('left', 'center', 'right')

# 局面 (デバッグ表示・教師ルール用)
STRAIGHT, ENTRY, APEX, EXIT, LOST = 'STRAIGHT', 'ENTRY', 'APEX', 'EXIT', 'LOST'


@dataclass
class SixLaneParams:
    # --- 知覚 ---
    stations: tuple = (3.0, 6.0, 9.5)  # [m] 曲率を評価する前方距離 (近/中/遠)
    station_margin: float = -0.5  # [m] 点群がステーションの先までこれだけ届いていないと評価しない (端点の外挿を避ける)
    min_point_x: float = 1.0  # [m] これより手前の点は使わない (画像最下端は検出が段差状に乱れる)
    bin_width: float = 0.5  # [m] 点群を前方距離のビンで平均してからフィット (画像の行で等間隔 = 近くに密集, を均す)
    min_points: int = 4  # 最低ビン数
    min_span: float = 2.5  # [m]
    cubic_ridge: float = 0.05
    kappa_alpha: tuple = (0.35, 0.3, 0.2)  # 曲率EMA係数 (15Hz, 近/中/遠. 遠いほどノイズが大きいので強く平滑化)
    kappa_decay: float = 0.9  # 観測なしフレームで0へ減衰
    x_pos: float = 1.5  # [m] 横位置を測る前方距離 (そこでの車線の向きで車軸位置へ戻す). 0 (車軸) で直接測ると
    #                     2次式を点群の手前まで外挿することになり, カーブ入口で切片がずれて横位置を大きく誤る
    # 現在の横位置 F の追跡: 白線の役割 (左/中央/右) の取り違えで F が1フレームで跳ぶのを弾く.
    # 車の横移動は 1 フレームでせいぜい 0.03 レーンなので, 予測から lateral_gate 以上ずれた観測は捨てる
    lateral_gate: float = 0.35  # [レーン]
    lateral_gain: float = 0.5
    lateral_resync_time: float = 3.0  # [s] ずれがこれだけ続いたら観測の方を信じ直す
    # --- 判断 (教師ルール・特徴量スケール. 学習時と推論時で共通) ---
    v_max: float = 1.5
    kappa_scale: float = 10.0  # NN入力 = kappa * kappa_scale
    kappa_straight: float = 0.015
    kappa_curve: float = 0.06
    home_lane: int = 6  # 直線で戻る外側レーン (左回り → 右端)
    # --- コミット層 ---
    switch_margin: float = 0.12  # 確率差がこれ以上で切替候補
    switch_frames: int = 5
    switch_frames_per_lane: int = 2  # 1レーン離れるごとに必要な連続フレームを増やす (大移動ほど慎重に)
    cone_x_min: float = -1.0
    cone_x_max: float = 8.0
    cone_clearance: float = 0.75  # [m] コーン中心とレーン中心の横距離がこれ未満なら塞がれている
    cone_block_factor: float = 0.02
    cone_pass_margin: float = 1.2  # [m] コーンが車体後端を抜けるまで塞がれたままにする距離 (カメラ死角対策)
    # --- 制御 ---
    lookahead_min: float = 2.0
    lookahead_max: float = 3.5
    lookahead_time: float = 1.5
    max_approach_angle: float = math.radians(22.0)  # レーン変更時の目標点への最大進入角
    edge_clearance: float = 0.75  # [m] レーン1/6 の目標は左右白線からこれ以上内側 (レーン中心は 0.58m で車体がはみ出しやすい)
    max_angular_speed: float = 1.2
    max_angular_accel: float = 4.0
    a_lat_max: float = 1.2
    v_min: float = 0.5
    accel: float = 0.8
    decel: float = 1.5
    conf_alpha: float = 0.1  # 速度に使う検出信頼度のEMA係数 (1フレームの欠落で速度を揺らさない)
    lost_timeout: float = 0.8  # [s] 白線を見失ってから停止へ


@dataclass
class LineObs:
    """1本の白線観測 (base_link). fit は y = c0 + c1 x + c2 x^2 (inferred=補完線)."""
    c: Sequence[float]
    x_min: float
    x_max: float
    detected: bool
    px: Optional[Sequence[float]] = None
    py: Optional[Sequence[float]] = None

    def y_at(self, x: float) -> float:
        return self.c[0] + self.c[1] * x + self.c[2] * x * x


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# 知覚: 曲率プロファイル
# ---------------------------------------------------------------------------
def fit_cubic(xs: Sequence[float], ys: Sequence[float], ridge: float) -> Optional[np.ndarray]:
    x = np.asarray(xs, float)
    y = np.asarray(ys, float)
    w = 1.0 / np.maximum(x, 1.0)
    A = np.stack([np.ones_like(x), x, x * x, x ** 3], axis=1) * w[:, None]
    AtA = A.T @ A
    AtA[2, 2] += ridge
    AtA[3, 3] += ridge * 10.0
    try:
        return np.linalg.solve(AtA, A.T @ (y * w))
    except np.linalg.LinAlgError:
        return None


def bin_points(px: Sequence[float], py: Sequence[float], p: SixLaneParams):
    """min_point_x より先の点を bin_width ごとに平均する (前方距離方向に均一な重みにする)."""
    bins: Dict[int, List[float]] = {}
    for x, y in zip(px, py):
        if x < p.min_point_x:
            continue
        b = bins.setdefault(int(math.floor(x / p.bin_width)), [0.0, 0.0, 0])
        b[0] += x
        b[1] += y
        b[2] += 1
    keys = sorted(bins)
    return [bins[k][0] / bins[k][2] for k in keys], [bins[k][1] / bins[k][2] for k in keys]


def cubic_curvature(a: np.ndarray, x: float) -> float:
    d1 = a[1] + 2 * a[2] * x + 3 * a[3] * x * x
    d2 = 2 * a[2] + 6 * a[3] * x
    return d2 / (1 + d1 * d1) ** 1.5


def measure_curvatures(lines: Dict[str, Optional[LineObs]], p: SixLaneParams) -> List[Optional[float]]:
    """各ステーションの符号付き曲率 (左カーブ正). 観測できないステーションは None."""
    sums = [0.0] * len(p.stations)
    wsum = [0.0] * len(p.stations)
    for role in ROLES:
        ln = lines.get(role)
        if ln is None or not ln.detected or ln.px is None:
            continue
        xs, ys = bin_points(ln.px, ln.py, p)
        if len(xs) < p.min_points:
            continue
        lo, hi = min(xs), max(xs)
        if hi - lo < p.min_span:
            continue
        a = fit_cubic(xs, ys, p.cubic_ridge)
        if a is None:
            continue
        for i, xs_ in enumerate(p.stations):
            if xs_ < lo - p.station_margin or xs_ > hi + p.station_margin:
                continue
            w = float(len(xs))
            sums[i] += w * cubic_curvature(a, xs_)
            wsum[i] += w
    return [sums[i] / wsum[i] if wsum[i] > 0 else None for i in range(len(p.stations))]


def fill_unobserved(meas: List[Optional[float]]) -> List[Optional[float]]:
    """見えないステーション (カーブで白線が視野外へ出た等) は, 手前で最後に見えた曲率が続くとみなす."""
    out, last = [], None
    for m in meas:
        last = m if m is not None else last
        out.append(last)
    return out


# ---------------------------------------------------------------------------
# 知覚: レーン座標
# ---------------------------------------------------------------------------
def lane_coordinate(lines: Dict[str, Optional[LineObs]], x: float, y: float) -> Optional[float]:
    """点 (x, y) のレーン座標 F (左白線=0, 中央線=3, 右白線=6. 範囲外は線形外挿)."""
    L, C, R = lines.get('left'), lines.get('center'), lines.get('right')
    if L is None or C is None or R is None:
        return None
    yl, yc, yr = L.y_at(x), C.y_at(x), R.y_at(x)
    if yl - yc < 0.3 or yc - yr < 0.3:
        return None
    if y >= yc:
        return 3.0 * (yl - y) / (yl - yc)
    return 3.0 + 3.0 * (yc - y) / (yc - yr)


def lane_y(lines: Dict[str, Optional[LineObs]], x: float, F: float) -> float:
    """レーン座標 F の x における横位置 y (lane_coordinate の逆)."""
    yl, yc, yr = lines['left'].y_at(x), lines['center'].y_at(x), lines['right'].y_at(x)
    if F <= 3.0:
        return yl + (yc - yl) * F / 3.0
    return yc + (yr - yc) * (F - 3.0) / 3.0


def sub_lane_width(lines, x: float, F: float) -> float:
    yl, yc, yr = lines['left'].y_at(x), lines['center'].y_at(x), lines['right'].y_at(x)
    return max(((yl - yc) if F <= 3.0 else (yc - yr)) / 3.0, 1e-3)


def lane_slope(lines, x: float, F: float) -> float:
    """レーン座標 F の平行線の x における傾き dy/dx (= 車に対する車線の向き)."""
    return (lane_y(lines, x + 0.25, F) - lane_y(lines, x - 0.25, F)) / 0.5


def vehicle_lane_coordinate(lines, x_eval: float) -> Optional[float]:
    """車軸 (base_link 原点) のレーン座標. 点群のある x_eval で車の正面の点のレーン座標を測り,
    そこでの車線の向き psi を使って車軸位置へ戻す (正面の点は車軸から x_eval*sin(psi) だけ横にずれている)."""
    F_p = lane_coordinate(lines, x_eval, 0.0)
    if F_p is None or x_eval == 0.0:
        return F_p
    psi = math.atan(lane_slope(lines, x_eval, F_p))
    return F_p - x_eval * math.sin(psi) / sub_lane_width(lines, x_eval, F_p)


def lane_of(F: float) -> int:
    return int(clamp(math.floor(F) + 1, 1, N_LANES))


# ---------------------------------------------------------------------------
# 判断: 特徴量・MLP・教師ルール
# ---------------------------------------------------------------------------
FEATURE_NAMES = ('v', 'kappa_near', 'kappa_mid', 'kappa_far', 'lateral', 'confidence')


def features(v, kappas, F, conf, p: SixLaneParams) -> np.ndarray:
    k = [kk * p.kappa_scale for kk in kappas]
    x = np.array([v / p.v_max, k[0], k[1], k[2], (F - 3.0) / 3.0, conf], float)
    return np.clip(x, -2.0, 2.0)


class LanePolicyNet:
    """入力6 → tanh隠れ層 → 6レーンlogit の MLP (six_lane_policy.json)."""

    def __init__(self, W1, b1, W2, b2):
        self.W1, self.b1 = np.asarray(W1, float), np.asarray(b1, float)
        self.W2, self.b2 = np.asarray(W2, float), np.asarray(b2, float)

    @classmethod
    def load(cls, path: str) -> 'LanePolicyNet':
        with open(path) as f:
            d = json.load(f)
        return cls(d['W1'], d['b1'], d['W2'], d['b2'])

    def forward(self, x: np.ndarray):
        h = np.tanh(self.W1 @ x + self.b1)
        z = self.W2 @ h + self.b2
        e = np.exp(z - z.max())
        return e / e.sum(), h


def _d(kappa: float, p: SixLaneParams) -> float:
    return clamp((abs(kappa) - p.kappa_straight) / (p.kappa_curve - p.kappa_straight), 0.0, 1.0)


def classify_phase(v, kappas, p: SixLaneParams):
    """教師ルールの局面判定. 戻り値 (phase, sign(+1=左カーブ), 目標レーン(連続値), 旋回強度)."""
    kn, km, kf = kappas
    dominant = max((kn, km, kf), key=abs)
    if abs(dominant) < p.kappa_straight:
        return STRAIGHT, 0, float(p.home_lane), 0.0
    s = 1 if dominant > 0 else -1
    same = lambda k: _d(k, p) if k * s > 0 else 0.0  # noqa: E731
    c_in = 0.6 * same(kn) + 0.4 * same(km)
    c_up = same(kf)
    outer = 6.0 if s > 0 else 1.0
    inner = 1.0 if s > 0 else 6.0
    sf = clamp(0.5 + 0.5 * v / p.v_max, 0.5, 1.0)
    if c_in >= 0.45 and c_up >= 0.35:
        depth = sf * min(1.0, 1.2 * c_in)
        return APEX, s, outer + (inner - outer) * depth, c_in
    if c_in >= 0.45:
        return EXIT, s, outer, c_in
    if c_up >= 0.35:
        return ENTRY, s, outer, c_up
    return STRAIGHT, s, float(p.home_lane), 0.0


def teacher_distribution(v, kappas, F, conf, p: SixLaneParams, sigma=0.6, stay_gain=1.5) -> np.ndarray:
    """教師 (アウト・イン・アウト規則) のレーン確率. 信頼度が低いほど現在レーン維持を好む."""
    _, _, target, _ = classify_phase(v, kappas, p)
    cur = clamp(F + 0.5, 1.0, 6.0)
    k = np.arange(1, N_LANES + 1, dtype=float)
    score = -((k - target) ** 2) / (2 * sigma * sigma) - (1.0 - conf) * stay_gain * np.abs(k - cur)
    e = np.exp(score - score.max())
    return e / e.sum()


# ---------------------------------------------------------------------------
# プランナー本体
# ---------------------------------------------------------------------------
@dataclass
class SixLaneState:
    kappas: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    target_lane: Optional[int] = None
    pending_lane: Optional[int] = None
    pending_count: int = 0
    v_cmd: float = 0.0
    omega_cmd: float = 0.0
    lost_time: float = 0.0
    conf_ema: float = 1.0
    block_hold: Dict[int, float] = field(default_factory=dict)  # レーン -> 残り走行距離 [m]
    F_filt: Optional[float] = None
    reject_time: float = 0.0


class SixLanePlanner:
    def __init__(self, net: LanePolicyNet, params: SixLaneParams = None):
        self.p = params or SixLaneParams()
        self.net = net
        self.reset()

    def reset(self):
        self.s = SixLaneState()
        self.last = {}

    def step(self, dt: float, lines: Optional[Dict[str, Optional[LineObs]]], v_meas: float,
             cones: Sequence[Sequence[float]] = (), reanchored: bool = False) -> Dict:
        """1制御周期. lines=None は観測なし (タイムアウト監視用). 戻り値は指令とデバッグ情報.
        reanchored: 白線の追跡側が根拠 (3本揃い・二重線) をもって役割を付け直したフレーム. 横位置の跳びを受け入れる."""
        p, s = self.p, self.s
        F_meas = vehicle_lane_coordinate(lines, p.x_pos) if lines else None
        if F_meas is None:
            s.lost_time += dt
            if s.lost_time >= p.lost_timeout:  # 短い欠落は直前の指令を維持, 続けば減速停止
                s.v_cmd = max(0.0, s.v_cmd - p.decel * dt)
                s.omega_cmd = self._rate(s.omega_cmd, 0.0, dt)
                s.F_filt = None  # 長く見失ったら横位置は観測から取り直す
            self.last = {**self.last, 'phase': LOST, 'v': s.v_cmd, 'omega': s.omega_cmd, 'lost_time': s.lost_time}
            return self.last
        s.lost_time = 0.0
        # 現在の横位置 (追跡済み). shift = 白線の役割取り違えによる観測のずれ [レーン].
        # 白線の形 (傾き・曲がり) は取り違えても平行なので使い続け, 横位置だけ shift で補正する.
        if reanchored:
            s.F_filt = None  # 追跡側の付け直しは根拠があるので, 取り違えとみなさず観測から取り直す
        F0, rejected = self._track_lateral(lines, F_meas, v_meas, dt)
        shift = F_meas - F0

        # 曲率プロファイル (EMA, 観測なしのステーションは減衰)
        meas = measure_curvatures(lines, p)
        filled = fill_unobserved(meas)
        for i, m in enumerate(filled):
            s.kappas[i] = s.kappas[i] + p.kappa_alpha[i] * (m - s.kappas[i]) if m is not None else s.kappas[i] * p.kappa_decay
        conf = sum(1 for r in ROLES if lines.get(r) is not None and lines[r].detected) / 3.0
        cur_lane = lane_of(F0)
        s.conf_ema += p.conf_alpha * (conf - s.conf_ema)

        # NN
        x = features(v_meas, s.kappas, F0, conf, p)
        probs, hidden = self.net.forward(x)
        probs = probs.copy()
        nn_probs = probs.copy()

        # コーンで塞がれたレーンを除外
        blocked = self._blocked_lanes(lines, cones, max(v_meas, 0.0) * dt, shift)
        for k in blocked:
            probs[k - 1] *= p.cone_block_factor
        probs /= probs.sum()

        # コミット (ヒステリシス)
        if s.target_lane is None:
            s.target_lane = cur_lane
        best = int(np.argmax(probs)) + 1
        if s.target_lane in blocked and best not in blocked:
            s.target_lane, s.pending_lane, s.pending_count = best, None, 0
        elif best != s.target_lane and probs[best - 1] - probs[s.target_lane - 1] > p.switch_margin:
            s.pending_count = s.pending_count + 1 if s.pending_lane == best else 1
            s.pending_lane = best
            if s.pending_count >= p.switch_frames + p.switch_frames_per_lane * (abs(best - s.target_lane) - 1):
                s.target_lane, s.pending_lane, s.pending_count = best, None, 0
        else:
            s.pending_lane, s.pending_count = None, 0

        # 制御: 目標レーン中心へ, 進入角を制限した注視点で Pure Pursuit
        v_now = max(v_meas, 0.0)
        Ld = clamp(v_now * p.lookahead_time, p.lookahead_min, p.lookahead_max)
        Ft = self._target_coordinate(lines, Ld, s.target_lane)
        y_cur = lane_y(lines, Ld, F_meas)  # 今の横位置を保った場合の注視点 (白線の平行線なので shift に依らない)
        y_tgt = lane_y(lines, Ld, Ft + shift)
        max_dy = Ld * math.tan(p.max_approach_angle)
        ty = y_cur + clamp(y_tgt - y_cur, -max_dy, max_dy)
        tx = Ld
        kappa_pp = 2.0 * ty / (tx * tx + ty * ty)

        kappa_path = max(abs(s.kappas[0]), abs(s.kappas[1]), abs(kappa_pp))
        v_target = min(p.v_max, math.sqrt(p.a_lat_max / max(kappa_path, 1e-3)))
        v_target = max(v_target, p.v_min) * (0.6 + 0.4 * s.conf_ema)
        if v_target > s.v_cmd:
            s.v_cmd = min(v_target, s.v_cmd + p.accel * dt)
        else:
            s.v_cmd = max(v_target, s.v_cmd - p.decel * dt)
        omega = clamp(max(v_now, s.v_cmd) * kappa_pp, -p.max_angular_speed, p.max_angular_speed)
        s.omega_cmd = self._rate(s.omega_cmd, omega, dt)

        phase, sign, teacher_target, intensity = classify_phase(v_meas, s.kappas, p)
        self.last = {
            'phase': phase, 'sign': sign, 'teacher_target': teacher_target, 'intensity': intensity,
            'F': F0, 'F_meas': F_meas, 'lateral_rejected': rejected, 'current_lane': cur_lane, 'target_lane': s.target_lane,
            'pending_lane': s.pending_lane, 'pending_count': s.pending_count,
            'kappas': list(s.kappas), 'kappas_meas': meas, 'confidence': conf,
            'features': x.tolist(), 'nn_probs': nn_probs.tolist(), 'probs': probs.tolist(),
            'blocked': sorted(blocked), 'lookahead': [tx, ty], 'v': s.v_cmd, 'omega': s.omega_cmd,
        }
        return self.last

    def _track_lateral(self, lines, F_meas: float, v: float, dt: float):
        """横位置 F を, 白線に対する車の向き (平行線の傾き) と速度で予測し, 観測で補正する.
        予測から lateral_gate 以上離れた観測は白線の取り違えとみなして捨てる. 戻り値 (F, 捨てたか)."""
        p, s = self.p, self.s
        if s.F_filt is None:
            s.F_filt, s.reject_time = F_meas, 0.0
            return F_meas, False
        F_pred = s.F_filt + lane_slope(lines, p.x_pos, F_meas) * v * dt / sub_lane_width(lines, p.x_pos, F_meas)  # 車線が左へ向いている (slope>0) = 車は右 (F 増) へずれていく
        r = F_meas - F_pred
        if abs(r) <= p.lateral_gate:
            s.F_filt, s.reject_time = F_pred + p.lateral_gain * r, 0.0
            return s.F_filt, False
        s.reject_time += dt
        if s.reject_time >= p.lateral_resync_time:
            s.F_filt, s.reject_time = F_meas, 0.0
            return F_meas, False
        s.F_filt = F_pred
        return F_pred, True

    def _target_coordinate(self, lines, x: float, lane: int) -> float:
        """目標レーンのレーン座標 (中心). 端のレーンは白線から edge_clearance 以上離す."""
        yl, yc, yr = lines['left'].y_at(x), lines['center'].y_at(x), lines['right'].y_at(x)
        f_min = self.p.edge_clearance / max((yl - yc) / 3.0, 1e-3)
        f_max = 6.0 - self.p.edge_clearance / max((yc - yr) / 3.0, 1e-3)
        return clamp(lane - 0.5, f_min, f_max)

    def _rate(self, prev, target, dt):
        step = self.p.max_angular_accel * max(dt, 0.0)
        return prev + clamp(target - prev, -step, step)

    def _blocked_lanes(self, lines, cones, travelled: float, shift: float = 0.0) -> set:
        """コーンで塞がれたレーン. 見えなくなった後も, そのコーンが車体後端を抜ける距離だけ
        走るまで塞がれたままにする (走行距離は車輪速の積分. 自己位置は使わない)."""
        p, hold = self.p, self.s.block_hold
        for k in list(hold):
            hold[k] -= travelled
            if hold[k] <= 0:
                del hold[k]
        for c in cones:
            cx, cy = float(c[0]), float(c[1])
            if cx < p.cone_x_min or cx > p.cone_x_max:
                continue
            for k in range(1, N_LANES + 1):
                if abs(lane_y(lines, cx, k - 0.5 + shift) - cy) < p.cone_clearance:
                    hold[k] = max(hold.get(k, 0.0), cx + p.cone_pass_margin)
        return set(hold)
