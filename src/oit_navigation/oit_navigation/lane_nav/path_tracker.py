"""
操舵・速度指令の計算.

    - 1 周目 (中央白線トラッキング): 中央線フィット上の前方注視点 (x_la, y_c(x_la)) に向かう
      円弧の曲率 kappa = 2 y / (x^2 + y^2) を使い omega = v * kappa.
    - 2 周目以降 (レーシングライン追従): 最適化したウェイポイント列を Catmull-Rom で密に補間し,
      最近傍点から弧長 lookahead 先の点を車両座標に変換して同じ式で omega を出す.
      目標速度は最近傍点の速度プロファイル値.

角速度はレート制限 (max_angular_accel) と上限 (max_angular_speed) をかける.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class TrackerParams:
    lookahead_min: float = 1.2
    lookahead_max: float = 2.5
    lookahead_time: float = 0.6        # lookahead = clip(v * time, min, max) + cross_track_gain * 横ずれ
    cross_track_lookahead_gain: float = 2.0  # ラインから離れているほど遠くを見て緩やかに合流する
    max_angular_speed: float = 1.5
    max_angular_accel: float = 4.0
    mapping_speed: float = 1.0         # 1 周目の速度 [m/s]
    curve_slowdown: float = 0.5        # 1 周目: 曲率に応じた減速係数
    curvature_filter_tau: float = 0.5  # 1 周目: 減速に使う曲率のローパス時定数 [s] (0 で無効)


def arc_curvature(x: float, y: float) -> float:
    d2 = x * x + y * y
    return 0.0 if d2 < 1e-6 else 2.0 * y / d2


def rate_limit(prev: float, target: float, max_rate: float, dt: float) -> float:
    step = max_rate * max(dt, 0.0)
    return prev + max(-step, min(step, target - prev))


def densify_closed(points: np.ndarray, values: np.ndarray, step: float = 0.2) -> Tuple[np.ndarray, np.ndarray]:
    """閉ループのウェイポイント列を Catmull-Rom スプラインで step [m] 程度に補間する."""
    n = len(points)
    out_p, out_v = [], []
    for i in range(n):
        p0, p1, p2, p3 = points[(i - 1) % n], points[i], points[(i + 1) % n], points[(i + 2) % n]
        seg = float(np.linalg.norm(p2 - p1))
        m = max(1, int(math.ceil(seg / step)))
        for k in range(m):
            t = k / m
            t2, t3 = t * t, t * t * t
            pt = 0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                        + (-p0 + 3 * p1 - 3 * p2 + p3) * t3)
            out_p.append(pt)
            out_v.append(values[i] + (values[(i + 1) % n] - values[i]) * t)
    return np.asarray(out_p), np.asarray(out_v)


class RacelineFollower:
    def __init__(self, points: np.ndarray, speeds: np.ndarray, params: TrackerParams, step: float = 0.2):
        self.p = params
        self.path, self.speed = densify_closed(points, speeds, step)
        seg = np.linalg.norm(np.roll(self.path, -1, axis=0) - self.path, axis=1)
        self.seg = seg
        self.idx: Optional[int] = None

    def nearest(self, x: float, y: float) -> int:
        n = len(self.path)
        if self.idx is None:
            d = np.hypot(self.path[:, 0] - x, self.path[:, 1] - y)
            self.idx = int(np.argmin(d))
        else:
            # 前回位置の前後だけ探す (周回の反対側に飛ばないように)
            win = np.arange(self.idx - 20, self.idx + 80) % n
            d = np.hypot(self.path[win, 0] - x, self.path[win, 1] - y)
            self.idx = int(win[int(np.argmin(d))])
        return self.idx

    def target(self, pose, v_now: float):
        """(target_speed, lookahead_point_vehicle_xy, nearest_index, cross_track) を返す."""
        x, y, yaw = pose
        i = self.nearest(x, y)
        cross = math.hypot(self.path[i, 0] - x, self.path[i, 1] - y)
        la = min(max(v_now * self.p.lookahead_time, self.p.lookahead_min), self.p.lookahead_max)
        la += self.p.cross_track_lookahead_gain * cross
        n = len(self.path)
        j, acc = i, 0.0
        while acc < la:
            acc += self.seg[j]
            j = (j + 1) % n
        dx, dy = self.path[j, 0] - x, self.path[j, 1] - y
        c, s = math.cos(yaw), math.sin(yaw)
        tx, ty = c * dx + s * dy, -s * dx + c * dy
        return float(self.speed[i]), (tx, ty), i, cross
