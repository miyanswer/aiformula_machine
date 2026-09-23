"""赤信号停止 (utils/traffic_light_stop.py) のテスト. ROS 不要.

    python3 -m pytest src/oit_navigation/test/test_traffic_light_stop.py
"""

import importlib.util
import os
import random

_PATH = os.path.join(os.path.dirname(__file__), '..', 'oit_navigation', 'utils', 'traffic_light_stop.py')
_spec = importlib.util.spec_from_file_location('traffic_light_stop', _PATH)
tls = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(tls)  # utils/__init__ (cv2) を読まずに単体で読む

DT = 0.05          # 制御周期 20Hz
DET_EVERY = 3      # 検出は約 6.7Hz (推論の遅い実機を想定)
BRAKE = 3.3        # [m/s^2] シミュレータの車両 (vehicle_physics.js BRAKE_FORCE_N / mass)
ACCEL = 2.2


def drive(stop, d0, v0=1.5, v_cmd=1.5, seconds=30.0, light=lambda t: 'red', noise=0.0, see_from=25.0, seed=0):
    """直線で信号機に向かって走る閉ループ. 返り値: (時刻, 距離, 速度) の列."""
    rng = random.Random(seed)
    d, v, t, log = d0, v0, 0.0, []
    for k in range(int(seconds / DT)):
        if k % DET_EVERY == 0 and 1.0 < d <= see_from:
            meas = d * (1.0 + rng.gauss(0.0, noise))
            color = light(t)
            stop.observe(t, red=meas if color == 'red' else None, green=meas if color == 'green' else None)
        vc, _ = stop.apply(t, DT, v, v_cmd, 0.3)
        v = min(vc, v + ACCEL * DT) if vc > v else max(vc, v - BRAKE * DT)
        d -= v * DT
        t += DT
        log.append((t, d, v))
    return log


def test_stops_between_5_and_10m():
    for d0 in (12.0, 15.0, 20.0, 25.0):
        for v0 in (0.8, 1.5, 3.0):
            for seed in range(3):
                stop = tls.TrafficLightStop()
                log = drive(stop, d0, v0, v_cmd=v0, seconds=40.0, noise=0.08, seed=seed)
                assert stop.state == tls.STOPPED, (d0, v0, seed, stop.status())
                d_final = log[-1][1]
                assert 5.0 <= d_final <= 10.0, (d0, v0, seed, d_final)
                assert log[-1][2] == 0.0


def test_single_false_red_is_ignored():
    stop = tls.TrafficLightStop()
    stop.observe(0.0, red=15.0)
    v, _ = stop.apply(0.05, 0.05, 1.5, 1.5, 0.2)
    assert stop.state == tls.NORMAL and v == 1.5
    stop.observe(2.0, red=15.0)   # 間が空いた 2 回目も「連続」ではない
    assert stop.state == tls.NORMAL


def test_green_resumes_after_stop():
    light = lambda t: 'red' if t < 15.0 else 'green'  # noqa: E731
    stop = tls.TrafficLightStop()
    log = drive(stop, 18.0, seconds=14.0, light=light)
    assert stop.state == tls.STOPPED
    d_stop = log[-1][1]
    more = []
    t0 = log[-1][0]
    d, v = d_stop, 0.0
    for k in range(int(6.0 / DT)):
        t = t0 + k * DT
        if k % DET_EVERY == 0:
            stop.observe(t, **({'green': d} if light(t) == 'green' else {'red': d}))
        vc, _ = stop.apply(t, DT, v, 1.5, 0.0)
        v = min(vc, v + ACCEL * DT) if vc > v else max(vc, v - BRAKE * DT)
        d -= v * DT
        more.append(v)
    assert stop.state == tls.NORMAL
    assert more[-1] > 1.0
    # 発進は resume_accel で滑らか (いきなり 1.5m/s にしない)
    first = next(i for i, x in enumerate(more) if x > 0.0)
    assert more[first] < 0.1


def test_red_lost_while_stopped_releases():
    stop = tls.TrafficLightStop()
    drive(stop, 15.0, seconds=12.0)
    assert stop.state == tls.STOPPED
    t = 12.0 + stop.p.red_release_time + 0.2
    stop.apply(t, DT, 0.0, 1.5, 0.0)
    assert stop.state == tls.RESUME


def test_red_first_seen_within_5m_passes_through():
    # 5m より近くで初めて赤を見たら止まらずに通過する
    stop = tls.TrafficLightStop()
    log = drive(stop, 4.8, v0=1.0, v_cmd=1.0, seconds=6.0, see_from=4.8)
    assert stop.state == tls.NORMAL
    assert min(v for _, _, v in log) == 1.0 and log[-1][1] < 0.0
    assert '通過' in stop.reason or stop.reason == '信号なし'
    # 1 フレーム目が 5m 以上なら, 2 フレーム目で 5m を割っていても止まる (初めて見た距離で決める)
    stop = tls.TrafficLightStop()
    stop.observe(0.0, red=5.3)
    stop.observe(0.15, red=4.9)
    assert stop.state == tls.APPROACH


def test_late_red_between_5m_and_target_still_stops():
    # 目標 7.0m より近い 6m で初めて気付いても 5m 以上なら止まる
    stop = tls.TrafficLightStop()
    log = drive(stop, 6.0, v0=1.0, v_cmd=1.0, seconds=5.0, see_from=6.0)
    assert stop.state == tls.STOPPED and 5.0 <= log[-1][1] <= 6.0


def test_omega_keeps_curvature_and_zero_when_stopped():
    stop = tls.TrafficLightStop()
    stop.observe(0.0, red=8.0)
    stop.observe(0.1, red=8.0)
    v, omega = stop.apply(0.15, DT, 1.5, 1.5, 0.6)
    assert v < 1.5 and abs(omega / v - 0.4) < 1e-9
    drive(stop, 8.0, seconds=6.0)
    assert stop.apply(7.0, DT, 0.0, 1.5, 0.6) == (0.0, 0.0)


def test_disabled_passes_through():
    stop = tls.TrafficLightStop(tls.TrafficLightStopParams(enabled=False))
    log = drive(stop, 15.0, seconds=6.0)
    assert stop.state == tls.NORMAL and log[-1][2] == 1.5
