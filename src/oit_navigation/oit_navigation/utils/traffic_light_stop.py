#!/usr/bin/env python3
"""
traffic_light_stop.py - 赤信号で信号機の手前 (既定 5〜10m, 目標 7.0m) に止まる速度制限 (ROS 非依存).

traffic_light_distance_node (traffic_light.pt + 画面占有率からの距離逆算) が出す
赤/青信号までの距離を受け取り, 走行ノード (lane_navigator / six_lane_planner) の最終 cmd_vel に
速度上限を掛ける. 走行方式には依存しない (どちらのノードも最後にこれを通す).

状態:
    NORMAL    : 制限なし
    APPROACH  : 赤を確認 (連続 red_confirm_frames 回). 残り距離 d から v <= sqrt(2 * decel * (d - stop_distance))
                で減速し, 目標距離で止まる. 検出の合間は車輪速 v * dt で距離を減らして補間する
    STOPPED   : 停止中 (v = omega = 0). 青を確認するか, 赤が red_release_time 見えなくなったら発進
    RESUME    : resume_accel で速度上限を戻し, 走行ノードの指令に追いついたら NORMAL

赤を初めて見た距離 (連続検出の 1 フレーム目) が min_trigger_distance (5m) より近ければ,
もう 5〜10m の範囲では止まれないので止まらずに通過する. 5m 以上なら stop_distance より近くても止まる.

web_simulator/js/traffic_light_stop.js と同一 (パラメータ・計算を変えたら両方直すこと).
"""

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

NORMAL = 'NORMAL'
APPROACH = 'APPROACH'
STOPPED = 'STOPPED'
RESUME = 'RESUME'


@dataclass
class TrafficLightStopParams:
    enabled: bool = True
    stop_distance: float = 7.0          # [m] 止まる目標 (信号機までの距離). 許容範囲の真ん中 (7.5) より少し近め:
                                        # 占有率からの距離は横にずれた位置から見ると短めに出る (シミュレータで最大 -1.5m) ため
    stop_distance_min: float = 5.0      # [m] 許容範囲 (状態表示・検証用)
    stop_distance_max: float = 10.0     # [m]
    detect_range: float = 25.0          # [m] これより遠い赤は無視
    min_trigger_distance: float = 5.0   # [m] 赤を初めて見たのがこれより近ければ通過 (止まっても 5m を割るため)
    decel: float = 0.6                  # [m/s^2] 計画減速度
    creep_speed: float = 0.2            # [m/s] 目標の手前 creep_gap 以上ある間の最低速度 (手前で止まり切らない)
    creep_gap: float = 0.4              # [m]
    stop_speed: float = 0.05            # [m/s] これ以下で目標に着いたら停止とみなす
    red_confirm_frames: int = 2         # 赤を連続で何回見たら減速を始めるか (誤検出対策)
    green_confirm_frames: int = 2       # 青を連続で何回見たら発進するか
    frame_gap: float = 0.6              # [s] これ以上間が空いたら「連続」を数え直す
    red_release_time: float = 3.0       # [s] 停止中に赤がこれだけ見えなければ発進 (消灯・誤検出)
    fuse_alpha: float = 0.5             # 観測距離と補間距離の混合 (1 = 観測のみ)
    resume_accel: float = 0.8           # [m/s^2] 発進時の速度上限の増加率


class TrafficLightStop:
    def __init__(self, params: Optional[TrafficLightStopParams] = None):
        self.p = params or TrafficLightStopParams()
        self.reset()

    def reset(self):
        self.state = NORMAL
        self.distance: Optional[float] = None   # 補間込みの信号機までの距離 [m]
        self.v_cap: Optional[float] = None
        self.red_count = 0
        self.green_count = 0
        self.last_red_t: Optional[float] = None
        self.last_green_t: Optional[float] = None
        self.last_red_distance: Optional[float] = None
        self.red_first_distance: Optional[float] = None   # 今の連続検出の 1 フレーム目の距離
        self.last_green_distance: Optional[float] = None
        self._obs_t: Optional[float] = None               # 最後に距離を観測値で更新した時刻
        self.stopped_distance: Optional[float] = None   # 停止した時点の距離 (検証用)
        self.reason = '信号なし'

    # ------------------------------------------------------------------ observations
    def observe(self, now: float, red: Optional[float] = None, green: Optional[float] = None):
        """1 フレーム分の検出結果 (見えなかった色は None). 距離はカメラ -> 信号機 [m]."""
        if red is not None:
            self._observe_red(now, red)
        if green is not None:
            self._observe_green(now, green)

    def _observe_red(self, now: float, d: float):
        p = self.p
        if d > p.detect_range:
            return
        self.red_count = self.red_count + 1 if self._recent(self.last_red_t, now) else 1
        if self.red_count == 1:
            self.red_first_distance = d
        self.last_red_t = now
        self.last_red_distance = d
        if self.state in (APPROACH, STOPPED):
            self.distance = d if self.distance is None else p.fuse_alpha * d + (1.0 - p.fuse_alpha) * self.distance
            self._obs_t = now
            self.green_count = 0
        elif self.red_count >= p.red_confirm_frames and self.red_first_distance >= p.min_trigger_distance and p.enabled:
            self.state = APPROACH
            self.distance = d
            self._obs_t = now
            self.green_count = 0
            self.stopped_distance = None

    def _observe_green(self, now: float, d: float):
        p = self.p
        if d > p.detect_range:
            return
        self.green_count = self.green_count + 1 if self._recent(self.last_green_t, now) else 1
        self.last_green_t = now
        self.last_green_distance = d
        if self.state in (APPROACH, STOPPED) and self.green_count >= p.green_confirm_frames:
            self._resume(now, '青信号を確認 → 発進')

    def _recent(self, t: Optional[float], now: float) -> bool:
        return t is not None and 0.0 <= now - t <= self.p.frame_gap

    def _resume(self, now: float, reason: str):
        self.state = RESUME
        self.reason = reason
        self.red_count = 0

    # ------------------------------------------------------------------ control
    def apply(self, now: float, dt: float, v_meas: float, v_cmd: float, omega_cmd: float) -> Tuple[float, float]:
        """走行ノードの指令 (v_cmd, omega_cmd) に信号の速度上限を掛ける. 曲率 (omega / v) は保つ."""
        p = self.p
        if not p.enabled:
            self.state, self.v_cap = NORMAL, None
            return v_cmd, omega_cmd
        if self.state in (APPROACH, STOPPED) and self.distance is not None:
            # 検出の合間は車輪速で補間. 直前に観測があればその時刻以降の移動分だけ引く
            moved_dt = dt if self._obs_t is None else max(0.0, min(dt, now - self._obs_t))
            self.distance -= max(v_meas, 0.0) * moved_dt
            self._obs_t = None

        if self.state == APPROACH:
            gap = self.distance - p.stop_distance
            cap = math.sqrt(2.0 * p.decel * max(gap, 0.0))
            if gap > p.creep_gap:
                cap = max(cap, p.creep_speed)
            self.v_cap = cap
            self.reason = f'赤信号 {self.distance:.1f}m 先 → {p.stop_distance:.1f}m 手前で停止するため減速'
            if gap <= 0.0 or (cap < p.creep_speed and v_meas <= p.stop_speed):
                if v_meas <= p.stop_speed:
                    self.state = STOPPED
                    self.stopped_distance = self.distance
                else:
                    self.v_cap = 0.0
        if self.state == STOPPED:
            self.v_cap = 0.0
            self.reason = f'赤信号で停止中 (信号機まで {self.distance:.1f}m)'
            if self.last_red_t is None or now - self.last_red_t > p.red_release_time:
                self._resume(now, f'赤信号が {p.red_release_time:.0f}秒見えない → 発進')
                self.v_cap = 0.0
        if self.state == RESUME:
            base = self.v_cap if self.v_cap is not None else max(v_meas, 0.0)
            self.v_cap = base + p.resume_accel * dt
            if self.v_cap >= v_cmd:
                self.state, self.v_cap, self.distance = NORMAL, None, None
                self.reason = '信号なし'
        if self.state == NORMAL:
            self.v_cap = None
            if self.last_red_t is not None and self.red_count > 0 and now - self.last_red_t <= p.frame_gap:
                if self.red_first_distance < p.min_trigger_distance:
                    self.reason = (f'赤信号を検出したが初検出が {self.red_first_distance:.1f}m '
                                   f'({p.min_trigger_distance:.0f}m 未満) → 止まらず通過')
                else:
                    self.reason = f'赤信号を検出 ({self.red_count}/{p.red_confirm_frames}回)'
            elif self.reason.startswith('赤信号を検出'):
                self.reason = '信号なし'
            return v_cmd, omega_cmd

        v = min(v_cmd, self.v_cap)
        if v_cmd > 1e-6:
            omega = omega_cmd * max(v, 0.0) / v_cmd
        else:
            omega = omega_cmd if v > 0.0 else 0.0
        return v, omega

    @property
    def active(self) -> bool:
        return self.state != NORMAL

    def status(self) -> Dict:
        r = lambda x: None if x is None else round(float(x), 3)  # noqa: E731
        return {
            'state': self.state, 'distance': r(self.distance), 'v_cap': r(self.v_cap),
            'red_distance': r(self.last_red_distance), 'green_distance': r(self.last_green_distance),
            'red_count': self.red_count, 'green_count': self.green_count,
            'stop_distance': self.p.stop_distance, 'stopped_distance': r(self.stopped_distance),
            'reason': self.reason,
        }
