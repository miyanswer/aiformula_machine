"""
走行ロジック本体 (ROS 非依存. ROS ノード lane_navigator_node と Web シミュレータの
js/lane_navigator.js がこれと同じ処理をする).

状態遷移:
    MAPPING   1 周目. 中央白線をトラッキングしながら左右境界を記録.
              周回検出 (または finish_mapping() の手動トリガ) で OPTIMIZING へ.
    OPTIMIZING 記録からコースマップを作り QP でレーシングラインを計算 (完了まで中央線走行を継続).
              async_optimize=True なら QP は呼び出し側 (ROS ノードの別スレッド) が
              pending_course_map() -> optimize_raceline() -> finish_optimization() で行う.
    RACING    2 周目以降. レーシングラインを追従. 白線観測をマップに合わせて自己位置 (x, y, 方位) を補正.
    STOPPED   白線ロスト / 最適化失敗など. 停止指令を出し続ける.
"""

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from .boundary_recorder import BoundaryRecorder, RecorderParams
from .course_map import CourseMap, LapDetectorParams, build_course_map, lap_completed, odom_to_map_correction
from .line_tracker import TrackedLines
from .path_tracker import RacelineFollower, TrackerParams, arc_curvature, rate_limit
from .raceline_qp import Raceline, RacelineParams, optimize_raceline

MAPPING, OPTIMIZING, RACING, STOPPED = "MAPPING", "OPTIMIZING", "RACING", "STOPPED"


@dataclass
class NavigatorParams:
    recorder: RecorderParams = field(default_factory=RecorderParams)
    lap: LapDetectorParams = field(default_factory=LapDetectorParams)
    raceline: RacelineParams = field(default_factory=RacelineParams)
    tracker: TrackerParams = field(default_factory=TrackerParams)
    heading_window: int = 15           # 方位ドリフト推定に使う中央線方位の平均フレーム数
    lines_timeout: float = 0.8         # 1 周目: これ以上白線が来なければ減速停止 [s]
    stop_decel: float = 1.5
    map_matching_gain: float = 0.1     # 2 周目: 白線観測による自己位置補正ゲイン (0 で無効)
    map_matching_max_error: float = 0.8  # これ以上離れた観測点は対応付けない [m]
    match_x_min: float = 1.0
    match_x_max: float = 6.0
    match_x_step: float = 1.0
    match_damping: tuple = (2.0, 2.0, 20.0)  # (x, y, yaw) のダンピング (観測できない方向を動かさない)
    # 1 フレームで動かす補正の上限. オドメトリのドリフトは数 cm/s 程度なので小さく抑え,
    # 白線の誤対応 (合流部の分岐線など) で推定位置が暴走しないようにする
    match_max_step_xy: float = 0.03    # [m / 更新]
    match_max_step_yaw: float = 0.005  # [rad / 更新]
    match_min_lines: int = 2           # これ未満の本数しか対応が取れなければ補正しない


@dataclass
class Command:
    v: float
    omega: float


class LaneNavigator:
    def __init__(self, params: NavigatorParams = NavigatorParams(), async_optimize: bool = False):
        self.p = params
        self.async_optimize = async_optimize
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self):
        self.state = MAPPING
        self.recorder = BoundaryRecorder(self.p.recorder)
        self.start_pose: Optional[Tuple[float, float, float]] = None
        self.start_s = 0.0
        self.yaw_unwrapped: Optional[float] = None
        self._last_yaw = 0.0
        self.yaw_drift = 0.0
        self._line_heading_buf = deque(maxlen=self.p.heading_window)  # 直近の中央線絶対方位
        self._start_heading_samples = []
        self.course_map: Optional[CourseMap] = None
        self.raceline: Optional[Raceline] = None
        self.follower: Optional[RacelineFollower] = None
        self.lap = 1
        self.cmd = Command(0.0, 0.0)
        self._curv_filt: Optional[float] = None   # 1 周目の減速用曲率 (ローパス後)
        self.corr = np.zeros(3)            # 2 周目の自己位置補正 odom -> map (tx, ty, theta)
        self._corr_init = np.zeros(3)      # RACING 開始時の corr (方位ドリフト補正した地図に今の姿勢を合わせる)
        self.last_lines_time: Optional[float] = None
        self.last_index: Optional[int] = None
        self.lap_start_s = 0.0
        self._s_now = 0.0
        self.message = ""

    def load_map(self, course_map: CourseMap):
        """保存済みマップから直接 RACING を始める (開始位置・向きは 1 周目の開始時と同じにすること)."""
        self.course_map = course_map
        self.start_pose = tuple(course_map.start_pose)
        self._corr_init = np.zeros(3)      # 1 周目の開始姿勢から再開するので odom = map
        self._start_racing()

    def finish_mapping(self, pose, s: float) -> bool:
        """手動で 1 周目を終了してマップを確定する (周回検出がドリフトで成立しない場合用)."""
        if self.state != MAPPING or len(self.recorder.samples) < 5:
            return False
        self._build_and_optimize()
        return True

    # ------------------------------------------------------------------ main
    def step(self, now: float, dt: float, pose, v_meas: float, omega_meas: float, s: float,
             lines: Optional[TrackedLines] = None) -> Command:
        """now: 時刻 [s], pose: odom の (x, y, yaw), s: 走行距離, lines: 新しい白線観測 (無ければ None)."""
        if lines is not None and lines.lines.get("center") is not None:
            self.last_lines_time = now
        self._s_now = s

        if self.state == MAPPING:
            cmd = self._step_mapping(now, dt, pose, v_meas, omega_meas, s, lines)
        elif self.state == OPTIMIZING:
            if lines is not None:
                self._center_fit = lines.lines.get("center")
            cmd = self._center_tracking(now, dt, v_meas)
        elif self.state == RACING:
            cmd = self._step_racing(dt, pose, v_meas, lines, s)
        else:
            cmd = self._stop(dt)
        self.cmd = cmd
        return cmd

    # ------------------------------------------------------------------ lap 1
    def _step_mapping(self, now, dt, pose, v_meas, omega_meas, s, lines) -> Command:
        tp = self.p.tracker
        # ROS の odom 方位は ±pi で折り返すので, 周回数を数えられるよう連続値にする
        if self.yaw_unwrapped is None:
            self.yaw_unwrapped = pose[2]
        else:
            self.yaw_unwrapped += math.atan2(math.sin(pose[2] - self._last_yaw), math.cos(pose[2] - self._last_yaw))
        self._last_yaw = pose[2]

        if self.start_pose is None:
            if lines is None or lines.lines.get("center") is None:
                return self._stop(dt)
            self.start_pose = tuple(pose)
            self.start_s = s

        # 中央線の絶対方位 (= 車両方位 + 車両から見た中央線の向き). スタート直後と 1 周後で比べる
        if lines is not None and lines.detected.get("center"):
            h = self.yaw_unwrapped + lines.lines["center"].heading_at(0.5)
            self._line_heading_buf.append(h)
            if len(self._start_heading_samples) < self.p.heading_window:
                self._start_heading_samples.append(h)

        if lines is not None:
            L, C, R = lines.lines.get("left"), lines.lines.get("center"), lines.lines.get("right")
            xr = self.p.recorder.x_rec
            self.recorder.update(
                s, pose, v_meas, omega_meas,
                None if L is None else float(L.y_at(xr)),
                None if R is None else float(R.y_at(xr)),
                bool(lines.detected.get("left")), bool(lines.detected.get("right")),
                None if C is None else C.curvature_at(xr),
            )
            self._center_fit = C

        if lap_completed(self.recorder.samples, pose, s, self.start_pose, self.start_s, self.p.lap):
            if len(self._start_heading_samples) >= 3 and len(self._line_heading_buf) >= 3:
                # 1 周後の中央線の絶対方位 - スタート時 - 2pi*周回 = ジャイロの方位ドリフト
                # (スタート直後 / 周回直前それぞれ数フレームの中央値でノイズを抑える)
                turned = float(np.median(self._line_heading_buf)) - float(np.median(self._start_heading_samples))
                k = round(turned / (2.0 * math.pi))
                if k != 0:
                    self.yaw_drift = turned - 2.0 * math.pi * k
            self._build_and_optimize()
            if self.state == RACING:
                return self._step_racing(dt, pose, v_meas, lines, s)

        cmd = self._center_tracking(now, dt, v_meas)
        if self.state == MAPPING and self.message != "白線ロスト: 減速停止":
            self.message = f"1周目 記録中: 断面 {len(self.recorder.samples)} 点 / 間隔 {self.recorder.next_spacing():.1f}m"
        return cmd

    def _center_tracking(self, now, dt, v_meas) -> Command:
        """中央白線のレーントラッキング (1 周目 / QP 計算待ちの間)."""
        tp = self.p.tracker
        if self.last_lines_time is None or now - self.last_lines_time > self.p.lines_timeout:
            self.message = "白線ロスト: 減速停止"
            return self._stop(dt)

        C = getattr(self, "_center_fit", None)
        if C is None:
            return self._stop(dt)
        la = min(max(v_meas * tp.lookahead_time, tp.lookahead_min), tp.lookahead_max)
        la = max(la, C.x_min)
        kappa = arc_curvature(la, float(C.y_at(la)))
        # フレーム毎の中央線フィットの曲率は検出ノイズで揺れるため、そのまま速度に
        # すると目標速度が毎フレーム上下し前後にガクガクする。曲率をローパスし、
        # 速度も 2 周目と同じ加減速制限 (raceline.a_accel / a_decel) で変化させる。
        curv = abs(float(C.curvature_at(la)))
        if self._curv_filt is None or tp.curvature_filter_tau <= 0.0:
            self._curv_filt = curv
        else:
            self._curv_filt += (curv - self._curv_filt) * min(1.0, dt / tp.curvature_filter_tau)
        v_target = tp.mapping_speed / (1.0 + tp.curve_slowdown * self._curv_filt * 4.0)
        rp = self.p.raceline
        v = rate_limit(self.cmd.v, v_target, rp.a_accel if v_target > self.cmd.v else rp.a_decel, dt)
        omega = max(-tp.max_angular_speed, min(tp.max_angular_speed, v * kappa))
        omega = rate_limit(self.cmd.omega, omega, tp.max_angular_accel, dt)
        return Command(v, omega)

    def _build_and_optimize(self):
        self.state = OPTIMIZING
        try:
            self.course_map = build_course_map(self.recorder.samples, self.start_pose, self.p.lap,
                                               self.yaw_drift, self.p.recorder.x_rec)
            self._corr_init = odom_to_map_correction(self.recorder.samples, self.p.lap, self.yaw_drift)
            if self.async_optimize:
                self.message = f"QP 計算中: 断面 {len(self.course_map.left)} 点"
                return
            self._start_racing()
        except Exception as e:  # noqa: BLE001 - 失敗したら止まる
            self.fail(f"レーシングライン生成失敗: {e}")

    def pending_course_map(self) -> Optional[CourseMap]:
        """async_optimize 時: QP を解くべきコースマップ (OPTIMIZING 中のみ)."""
        return self.course_map if self.state == OPTIMIZING else None

    def finish_optimization(self, raceline: Raceline):
        if self.state == OPTIMIZING:
            self._start_racing(raceline)

    def fail(self, message: str):
        self.state = STOPPED
        self.message = message

    def _start_racing(self, raceline: Optional[Raceline] = None):
        m = self.course_map
        self.raceline = raceline if raceline is not None else optimize_raceline(m.left, m.right, self.p.raceline)
        self.follower = RacelineFollower(self.raceline.points, self.raceline.speed, self.p.tracker)
        self.corr = self._corr_init.copy()
        self.state = RACING
        self.lap = 2
        self.last_index = None
        self.lap_start_s = self._s_now
        self.message = f"2周目以降: ウェイポイント {len(self.raceline.points)} 点 / 1周 {self.raceline.length:.1f}m"

    # ------------------------------------------------------------------ lap 2+
    def map_pose(self, pose):
        tx, ty, th = self.corr
        c, s = math.cos(th), math.sin(th)
        return (tx + c * pose[0] - s * pose[1], ty + s * pose[0] + c * pose[1], pose[2] + th)

    def _step_racing(self, dt, pose, v_meas, lines, s_now: float) -> Command:
        tp = self.p.tracker
        if lines is not None and self.p.map_matching_gain > 0.0:
            self._map_matching(pose, lines)
        mp = self.map_pose(pose)
        v_target, (tx, ty), idx, _ = self.follower.target(mp, v_meas)
        # 周回数: 最近傍点がライン末尾から先頭へ戻ったら +1 (開始直後の誤カウント防止に 1 周の 7 割以上走行が条件)
        if (self.last_index is not None and idx < self.last_index - len(self.follower.path) // 2
                and s_now - self.lap_start_s > 0.7 * self.raceline.length):
            self.lap += 1
            self.lap_start_s = s_now
        self.last_index = idx
        v = rate_limit(self.cmd.v, v_target, self.p.raceline.a_accel if v_target > self.cmd.v
                       else self.p.raceline.a_decel, dt)
        omega = max(-tp.max_angular_speed, min(tp.max_angular_speed, max(v, 0.3) * arc_curvature(tx, ty)))
        omega = rate_limit(self.cmd.omega, omega, tp.max_angular_accel, dt)
        return Command(v, omega)

    def _map_matching(self, pose, lines: TrackedLines):
        """白線観測 (実検出のみ) をマップの境界線に点-線 ICP で合わせ, 自己位置 (x, y, 方位) を補正する.

        直線部では進行方向の位置は観測できない (白線が平行) ので, ダンピング付き最小二乗で
        観測できない方向は動かさない. 1 フレームで補正するのは gain 分だけ (ノイズ対策).
        """
        m = self.course_map
        pp = self.p
        mp = self.map_pose(pose)
        c, s = math.cos(mp[2]), math.sin(mp[2])
        J, r, w = [], [], []
        lines_used = 0
        for role, poly, weight in (("left", m.left, 1.0), ("right", m.right, 1.0), ("center", m.center, 0.5)):
            fit = lines.lines.get(role)
            if fit is None or not lines.detected.get(role):
                continue
            x_lo, x_hi = max(fit.x_min, pp.match_x_min), min(fit.x_max, pp.match_x_max)
            n_before = len(r)
            for x in np.arange(x_lo, x_hi + 1e-6, pp.match_x_step):
                y = float(fit.y_at(x))
                px, py = mp[0] + c * x - s * y, mp[1] + s * x + c * y
                hit = _nearest_segment(poly, px, py)
                if hit is None:
                    continue
                n, dist = hit
                if abs(dist) > pp.map_matching_max_error:
                    continue
                J.append([n[0], n[1], n[0] * -(py - mp[1]) + n[1] * (px - mp[0])])
                r.append(dist)
                w.append(weight)
            if len(r) > n_before:
                lines_used += 1
        if len(r) < 3 or lines_used < pp.match_min_lines:
            return
        J, r, w = np.asarray(J), np.asarray(r), np.asarray(w)
        H = J.T @ (J * w[:, None]) + np.diag(pp.match_damping)
        delta = -np.linalg.solve(H, J.T @ (w * r)) * pp.map_matching_gain
        delta[:2] = np.clip(delta[:2], -pp.match_max_step_xy, pp.match_max_step_xy)
        delta[2] = float(np.clip(delta[2], -pp.match_max_step_yaw, pp.match_max_step_yaw))
        new = (mp[0] + delta[0], mp[1] + delta[1], mp[2] + delta[2])
        # odom -> map の補正 (SE2) を, 補正後の map 姿勢になるように更新
        th = new[2] - pose[2]
        ct, st = math.cos(th), math.sin(th)
        self.corr = np.array([new[0] - (ct * pose[0] - st * pose[1]),
                              new[1] - (st * pose[0] + ct * pose[1]), th])

    def apply_external_correction(self, dx: float, dy: float, dyaw: float, damping: float = 0.15):
        """外部 (コーンのランドマーク照合など) からの自己位置補正を少しずつ足す (js applyExternalCorrection)."""
        self.corr = self.corr + damping * np.array([dx, dy, dyaw], float)

    def _stop(self, dt) -> Command:
        v = max(0.0, self.cmd.v - self.p.stop_decel * dt)
        return Command(v, 0.0 if v == 0.0 else self.cmd.omega * 0.9)

    # ------------------------------------------------------------------ status
    def status(self) -> dict:
        return {
            "state": self.state, "lap": self.lap,
            "samples": len(self.recorder.samples),
            "spacing": round(self.recorder.next_spacing(), 2),
            "kappa": round(self.recorder.kappa, 3),
            "v": round(self.cmd.v, 2), "omega": round(self.cmd.omega, 3),
            "correction": [round(float(v), 3) for v in self.corr],
            "yaw_drift": round(self.yaw_drift, 4),
            "message": self.message,
        }


def _nearest_segment(poly: np.ndarray, px: float, py: float):
    """閉ポリラインの最近傍線分への (単位法線, 符号付き距離) を返す."""
    a = poly
    b = np.roll(poly, -1, axis=0)
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(axis=1), 1e-12)
    t = np.clip(((px - a[:, 0]) * ab[:, 0] + (py - a[:, 1]) * ab[:, 1]) / L2, 0.0, 1.0)
    qx = a[:, 0] + t * ab[:, 0]
    qy = a[:, 1] + t * ab[:, 1]
    d2 = (px - qx) ** 2 + (py - qy) ** 2
    i = int(np.argmin(d2))
    L = math.sqrt(L2[i])
    if L < 1e-6:
        return None
    n = np.array([-ab[i, 1] / L, ab[i, 0] / L])
    return n, float(n[0] * (px - a[i, 0]) + n[1] * (py - a[i, 1]))
