"""コーン回避 (lane_nav/cone_avoidance.py = web_simulator/js/cone_avoidance.js) のテスト. ROS 不要.

    python3 -m pytest src/oit_navigation/test/test_cone_avoidance.py
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))

from oit_navigation.lane_nav import (  # noqa: E402
    LaneNavigator, LineTracker, LineTrackerParams, NavigatorParams, RACING,
)
from oit_navigation.lane_nav.cone_avoidance import (  # noqa: E402
    DEFLECT_CLEARANCE, KEEP_OUT_RADIUS, NavigatorConeAvoidance, ReactiveAvoider, cluster_cones, deflect_raceline_around_cones,
)
from lane_nav_sim import lateral_error_to_center, make_track, observe  # noqa: E402


def test_reactive_steers_away_and_slows():
    av = ReactiveAvoider()
    # 進路のやや左にあるコーン -> 右へ抜ける (負のバイアス)
    for _ in range(20):
        v, w = av.step(0.0, 0.05, 1.5, 0.0, [(3.0, 0.2)])
    assert av.active and av.locked_sign == -1 and w < -0.2 and v == 1.5
    # 近い (1.8m 未満) と減速
    v, w = av.step(0.0, 0.05, 1.5, 0.0, [(1.5, 0.2)])
    assert v == pytest.approx(0.6)
    # 進路から外れたコーン (|y| >= 1.0) は回避しない, バイアスは滑らかに 0 へ戻る
    b0 = av.bias
    v, w = av.step(0.0, 0.05, 1.5, 0.0, [(3.0, 1.5)])
    assert not av.active and abs(av.bias) < abs(b0) and abs(av.bias - b0) <= 4.0 * 0.05 + 1e-9


def test_clusters_link_cones_within_two_radii():
    cl = cluster_cones([(3.0, 0.0), (4.5, 0.3), (9.0, 0.0)])
    assert sorted(len(c['cones']) for c in cl) == [1, 2]


def test_deflection_pushes_raceline_out_of_keep_out_circle():
    # 閉曲線にするため折り返し (x: 0->20 を y=0 で, 戻りは y=-8), 点は 3m おき (QP の直線部のように疎)
    xs = np.arange(0.0, 21.0, 3.0)
    pts = np.array([(x, 0.0) for x in xs] + [(x, -8.0) for x in xs[::-1]])
    out, _ = deflect_raceline_around_cones(pts, np.ones(len(pts)), [(10.0, 0.3)])
    d = np.hypot(out[:, 0] - 10.0, out[:, 1] - 0.3)
    assert d.min() >= DEFLECT_CLEARANCE - 0.02        # 細かくした経路のどこもコーンから DEFLECT_CLEARANCE 以内に入らない
    assert DEFLECT_CLEARANCE > KEEP_OUT_RADIUS
    far = np.hypot(out[:, 0] - 10.0, out[:, 1] - 0.3) > 6.0    # 押し出しを配るのは前後 DEFLECT_WINDOW (4.5m) まで
    mid = far & (out[:, 1] > -4) & (out[:, 0] > 3) & (out[:, 0] < 15.5)   # 折り返しの角 (スプラインのはみ出し) は除く
    assert np.abs(out[mid, 1]).max() < 1e-6   # 離れた所は動かさない
    # 押し出した経路は滑らか (隣り合う点の向きの変化が小さい)
    t = np.diff(out, axis=0)
    ang = np.abs(np.diff(np.unwrap(np.arctan2(t[:, 1], t[:, 0]))))
    assert ang[(out[1:-1, 1] > -4) & (out[1:-1, 0] > 3) & (out[1:-1, 0] < 15.5)].max() < 0.25


def run_lap_with_cones(cones_true, seconds=260.0, dt=0.05, seed=1, avoid=True, bias=0.0003, yaw_corr=False):
    """lane_nav_sim と同じ合成コースで, lane_navigator ノードと同じ段取り (NavigatorConeAvoidance) を回す."""
    rng = np.random.default_rng(seed)
    track = make_track()
    params = NavigatorParams()
    params.lap.yaw_drift_correction = yaw_corr
    nav = LaneNavigator(params)
    tracker = LineTracker(LineTrackerParams(lane_width_init=1.75))
    ca = NavigatorConeAvoidance(reactive=avoid, deflect=avoid, landmark=avoid)
    true = np.array([2.0, 0.1, 0.0])
    odom = true.copy()
    v = w = s = t = 0.0
    min_clear = {RACING: math.inf, 'MAPPING': math.inf}
    max_lat = 0.0
    k = 0
    while t < seconds:
        lines = tracker.update(observe(track, true, rng))
        c, sn = math.cos(true[2]), math.sin(true[2])
        cones = []
        for cx, cy in cones_true:   # カメラのコーン検出 (前方 0.3〜8m, |y|<3, 5cm のノイズ)
            dx, dy = cx - true[0], cy - true[1]
            x, y = c * dx + sn * dy, -sn * dx + c * dy
            if 0.3 < x < 8.0 and abs(y) < 3.0:
                cones.append((x + rng.normal(0, 0.05), y + rng.normal(0, 0.05)))
        new = k % 2 == 0                             # 検出は制御の半分の周期
        if new:
            ca.observe(nav, s, tuple(odom), cones)
        v_meas = v * 1.01 + rng.normal(0, 0.01)
        w_meas = w + bias + rng.normal(0, 0.005)
        cmd = nav.step(t, dt, tuple(odom), v_meas, w_meas, s, lines)
        vc, wc, _ = ca.step(t, dt, nav, tuple(odom), cmd.v, cmd.omega, cones, new, params.lap, params.tracker)
        v += (vc - v) * min(1.0, dt / 0.2)
        w += (wc - w) * min(1.0, dt / 0.1)
        true[0] += v * math.cos(true[2] + 0.5 * w * dt) * dt
        true[1] += v * math.sin(true[2] + 0.5 * w * dt) * dt
        true[2] += w * dt
        odom[0] += v_meas * math.cos(odom[2] + 0.5 * w_meas * dt) * dt
        odom[1] += v_meas * math.sin(odom[2] + 0.5 * w_meas * dt) * dt
        odom[2] += w_meas * dt
        s += abs(v_meas) * dt
        t += dt
        k += 1
        key = RACING if nav.state == RACING else 'MAPPING'
        for cx, cy in cones_true:
            min_clear[key] = min(min_clear[key], math.hypot(cx - true[0], cy - true[1]))
        if k % 10 == 0:
            max_lat = max(max_lat, float(lateral_error_to_center(track, true[None, :2])[0]))
    min_clear['max_lateral'] = max_lat
    return nav, ca, min_clear


@pytest.mark.parametrize("bias,yaw_corr", [(0.0003, False), (-0.0003, False), (0.001, True), (-0.001, True)])
def test_qp_navigation_avoids_cones_on_both_laps(bias, yaw_corr):
    """1 周目 (中央線走行) は反応的回避, 2 周目以降はコーン地図で押し出したレーシングラインで, どのコーンにも
    車体が触れない (コーン中心から 0.5m = 車体半幅 0.35m + コーン半径 0.15m 以上離れる).
    方位ドリフト補正ありでも, コーン地図が境界地図と同じ補正を受けて正しい位置に載る."""
    cones = [(15.0, 0.0), (16.0, 24.4), (38.5, 13.0)]
    nav, ca, clear = run_lap_with_cones(cones, bias=bias, yaw_corr=yaw_corr)
    assert nav.state == RACING and nav.lap >= 2
    assert len(ca.cone_map) == len(cones)            # 同じコーンを名寄せして 3 個
    assert ca.deflected_points is not None
    assert clear['MAPPING'] > 0.5 and clear[RACING] > 0.5, clear
    assert clear['max_lateral'] < 1.55, clear    # コーンを避けてもコースの中 (test_lane_nav の周回テストと同じ基準)
    # コーン地図はレーシングラインと同じ座標: 実際に追従する (細かくした) 経路がどのコーン地図点からも離れている
    path = nav.follower.path
    for cx, cy in ca.cone_map:
        assert np.hypot(path[:, 0] - cx, path[:, 1] - cy).min() > DEFLECT_CLEARANCE - 0.05


@pytest.mark.parametrize("bias", [0.001, -0.001])
def test_cone_clearance_is_seed_robust_under_heavy_gyro_bias(bias):
    """大きなジャイロバイアス + 方位ドリフト補正で, 乱数シードを変えても 2 周目にコーンへ近づかない.

    以前は方位ドリフト補正した地図 (ドリフトを除いた座標) に, 1 周分ドリフトした odom 姿勢をそのまま載せて
    2 周目を始めていたため, 開始時に自己位置が ~2m / 0.15rad ずれ, 直線では白線照合で進行方向のずれが取れず
    (ランドマーク照合のゲート 1.0m の外), 押し出したラインが前後にずれてコーンから 0.03m まで近づくシードがあった.

    1 周目の周回検出 (odom が開始点の close_radius=2.5m 以内に戻る) は, 0.001 rad/s では 1 周後の odom の
    ずれが 2.0-2.8m になり成立しないシードがある (コーン回避とは別の既知の限界. 実機は停止中にバイアスを推定するので
    残差はこの 1/10 程度で, 成立しなければ finish_mapping で手動確定する). そこで判定を分け,
    周回できたシードでは例外なく厳密に (0.5m) 判定し, 周回できたシードが十分あること (判定が空にならないこと) を確かめる.
    """
    cones = [(15.0, 0.0), (16.0, 24.4), (38.5, 13.0)]
    closed = 0
    for seed in range(1, 7):
        nav, ca, clear = run_lap_with_cones(cones, bias=bias, yaw_corr=True, seed=seed)
        if nav.state != RACING:
            continue
        closed += 1
        assert len(ca.cone_map) == len(cones), seed
        assert clear['MAPPING'] > 0.5 and clear[RACING] > 0.5, (seed, clear)
        assert clear['max_lateral'] < 1.55, (seed, clear)
    assert closed >= 4, closed


def test_racing_starts_in_yaw_corrected_map_frame():
    """方位ドリフト補正を掛けたとき, 2 周目開始時の odom -> map 補正は最後の記録断面の odom 姿勢を
    補正後の姿勢へ移す (補正を掛けないときは恒等変換)."""
    from oit_navigation.lane_nav.boundary_recorder import BoundarySample
    from oit_navigation.lane_nav.course_map import LapDetectorParams, corrected_poses, odom_to_map_correction
    # 半径 10m の円を 1 周. odom の方位は走行距離に比例して 0.1 rad ドリフトしている
    drift = 0.1
    samples, pose = [], np.array([0.0, 0.0, 0.0])
    n, r = 60, 10.0
    ds = 2 * math.pi * r / n
    for i in range(n + 1):
        samples.append(BoundarySample(i * ds, tuple(pose), (0.0, 0.0), (0.0, 0.0), 0.1, True, True))
        yaw_rate = 1.0 / r + drift / (n * ds)
        pose = pose + np.array([ds * math.cos(pose[2] + 0.5 * yaw_rate * ds), ds * math.sin(pose[2] + 0.5 * yaw_rate * ds),
                                yaw_rate * ds])
    p = LapDetectorParams(yaw_drift_correction=True)
    corr = odom_to_map_correction(samples, p, drift)
    nav = LaneNavigator(NavigatorParams())
    nav.corr = corr
    mapped = np.array(nav.map_pose(samples[-1].pose))
    assert np.allclose(mapped, corrected_poses(samples, drift)[-1], atol=1e-9)
    assert corr[2] == pytest.approx(-drift)
    assert np.hypot(mapped[0], mapped[1]) < 0.2        # ドリフトを除くと 1 周後はほぼ開始点に戻る
    assert np.hypot(samples[-1].pose[0], samples[-1].pose[1]) > 0.5   # odom のままではずれている
    assert np.all(odom_to_map_correction(samples, LapDetectorParams(), drift) == 0.0)


def test_without_avoidance_the_car_hits_the_centre_cone():
    """比較: 回避なしだと 1 周目 (中央線の上を走る) で中央線上のコーンに当たる (テストが意味を持つことの確認)."""
    _, _, clear = run_lap_with_cones([(15.0, 0.0)], seconds=40.0, avoid=False)
    assert clear['MAPPING'] < 0.3
