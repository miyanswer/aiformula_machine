"""
1 周目: 走行しながら左右境界点 (x_L, y_L), (x_R, y_R) をオドメトリ座標 (odom) に「落としていく」.

    - 記録位置は車両前方 x_rec [m] (近すぎると画角外, 遠すぎると投影誤差が大きい).
      左右境界の横位置は直近 median_window フレームの中央値でノイズを落とす.
    - 記録間隔は曲率で可変: 直線 (|kappa| <= kappa_straight) は ds_straight [m] 間隔,
      カーブ (|kappa| >= kappa_curve) は ds_curve [m] 間隔, その間は線形補間.
      加えて前回記録からの方位変化が max_heading_step を超えたら距離に関係なく記録する.
    - 1 点ごとに左右ペア (同じ断面) で記録するので, 2 周目の QP はこの断面ごとに
      「左境界 + alpha * (右境界 - 左境界)」の 1 変数でウェイポイントを置ける.
"""

import math
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class RecorderParams:
    x_rec: float = 2.0              # 記録する前方距離 [m]
    ds_straight: float = 3.0        # 直線での記録間隔 [m]
    ds_curve: float = 0.6           # カーブでの記録間隔 [m]
    kappa_straight: float = 0.03    # これ以下の曲率は直線扱い [1/m]
    kappa_curve: float = 0.20       # これ以上の曲率はカーブ扱い [1/m]
    max_heading_step: float = math.radians(12.0)
    median_window: int = 5
    kappa_alpha: float = 0.3        # 曲率推定のローパス
    min_speed_for_kappa: float = 0.3


@dataclass
class BoundarySample:
    s: float                        # 記録時の走行距離 [m]
    pose: Tuple[float, float, float]
    left: Tuple[float, float]       # odom 座標
    right: Tuple[float, float]
    kappa: float
    left_detected: bool
    right_detected: bool
    y_left: float = 0.0             # 記録に使った車両座標の横位置 (方位ドリフト補正で再計算に使う)
    y_right: float = 0.0


def spacing_for_curvature(kappa: float, p: RecorderParams) -> float:
    k = abs(kappa)
    if k <= p.kappa_straight:
        return p.ds_straight
    if k >= p.kappa_curve:
        return p.ds_curve
    t = (k - p.kappa_straight) / (p.kappa_curve - p.kappa_straight)
    return p.ds_straight + t * (p.ds_curve - p.ds_straight)


def vehicle_to_world(pose, x: float, y: float) -> Tuple[float, float]:
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return px + c * x - s * y, py + s * x + c * y


class BoundaryRecorder:
    def __init__(self, params: RecorderParams = RecorderParams()):
        self.p = params
        self.samples: List[BoundarySample] = []
        self._buf_l: deque = deque(maxlen=params.median_window)
        self._buf_r: deque = deque(maxlen=params.median_window)
        self.kappa = 0.0
        self._last_s: Optional[float] = None
        self._last_yaw: Optional[float] = None

    def reset(self):
        self.__init__(self.p)

    def next_spacing(self) -> float:
        return spacing_for_curvature(self.kappa, self.p)

    def update(
        self, s: float, pose, v: float, omega: float,
        y_left: Optional[float], y_right: Optional[float],
        left_detected: bool, right_detected: bool,
        line_kappa: Optional[float] = None,
    ) -> Optional[BoundarySample]:
        """1 フレーム分の観測を入れる. 記録したらその BoundarySample を返す.

        s:        走行距離 [m] (オドメトリ積算)
        pose:     (x, y, yaw) odom 座標
        y_left/y_right: 前方 x_rec での左右境界の横位置 [m] (車両座標, 左が正). 不明なら None.
        line_kappa: 中央線フィットの曲率 (あれば曲率推定に使う)
        """
        p = self.p
        # 曲率推定: ヨーレート/速度 と 白線フィットの曲率の大きい方 (カーブ手前から詰め始める)
        k_meas = abs(omega) / v if v >= p.min_speed_for_kappa else 0.0
        if line_kappa is not None:
            k_meas = max(k_meas, abs(line_kappa))
        self.kappa += p.kappa_alpha * (k_meas - self.kappa)

        if y_left is not None:
            self._buf_l.append(y_left)
        if y_right is not None:
            self._buf_r.append(y_right)
        if not self._buf_l or not self._buf_r:
            return None

        if self._last_s is not None:
            ds = s - self._last_s
            dyaw = abs(math.atan2(math.sin(pose[2] - self._last_yaw), math.cos(pose[2] - self._last_yaw)))
            if ds < self.next_spacing() and dyaw < p.max_heading_step:
                return None
            if ds < 0.2:  # 停止中の方位ドリフトで連続記録しない
                return None

        yl = float(np.median(self._buf_l))
        yr = float(np.median(self._buf_r))
        sample = BoundarySample(
            s=s, pose=tuple(pose),
            left=vehicle_to_world(pose, p.x_rec, yl),
            right=vehicle_to_world(pose, p.x_rec, yr),
            kappa=self.kappa,
            left_detected=left_detected, right_detected=right_detected,
            y_left=yl, y_right=yr,
        )
        self.samples.append(sample)
        self._last_s = s
        self._last_yaw = pose[2]
        return sample
