"""oit_navigation/6lane/six_lane_core.py の単体テスト (ROS 不要: python3 -m pytest test/test_six_lane.py)."""

import importlib
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
core = importlib.import_module('oit_navigation.6lane.six_lane_core')  # フォルダ名が数字始まりなので importlib
POLICY = os.path.join(os.path.dirname(__file__), '..', 'oit_navigation', '6lane', 'six_lane_policy.json')


def make_lines(offset=0.0, kappa=0.0, kappa_far=None, width=3.5, detected=(True, True, True), noise=0.0, rng=None):
    """車両が中央線から offset [m] 左にいるときの3本の白線. kappa_far を与えると 6m より先で曲率が変わる."""
    kf = kappa if kappa_far is None else kappa_far
    lines = {}
    for role, d, det in zip(core.ROLES, (width, 0.0, -width), detected):
        c = (d - offset, 0.0, 0.5 * kappa)
        xs = list(np.linspace(1.2, 11.5, 18))
        ys = [c[0] + 0.5 * kappa * x * x + 0.5 * (kf - kappa) * max(0.0, x - 6.0) ** 2
              + (rng.normal(0, noise) if rng is not None else 0.0) for x in xs]
        lines[role] = core.LineObs(c=c, x_min=1.2, x_max=11.5, detected=det,
                                   px=xs if det else None, py=ys if det else None)
    return lines


@pytest.fixture
def planner():
    return core.SixLanePlanner(core.LanePolicyNet.load(POLICY))


def test_lane_coordinate_roundtrip():
    lines = make_lines(offset=1.0, kappa=0.05)
    for F in (0.0, 0.5, 2.9, 3.0, 4.2, 6.0):
        for x in (0.0, 3.0, 8.0):
            assert core.lane_coordinate(lines, x, core.lane_y(lines, x, F)) == pytest.approx(F, abs=1e-9)
    # 中央線から 1.0m 左 = レーン座標 3 - 1.0/(3.5/3)
    F0 = core.lane_coordinate(lines, 0.0, 0.0)
    assert F0 == pytest.approx(3 - 1.0 / (3.5 / 3))
    assert core.lane_of(F0) == 3
    assert core.lane_of(-0.2) == 1 and core.lane_of(6.3) == 6


def test_curvature_measurement():
    p = core.SixLaneParams()
    k = core.measure_curvatures(make_lines(kappa=0.07), p)
    for x, v in zip(p.stations, k):  # y = 0.5 kappa x^2 の真の曲率は遠方ほど小さい
        assert v == pytest.approx(0.07 / (1 + (0.07 * x) ** 2) ** 1.5, abs=0.006)
    k = core.measure_curvatures(make_lines(kappa=0.0, kappa_far=0.08), p)
    assert abs(k[0]) < 0.02 and k[2] > 0.04  # 近くは直線, 遠くでカーブ
    assert core.fill_unobserved([0.05, None, None]) == [0.05, 0.05, 0.05]


def test_out_in_out(planner):
    """直線 (レーン6) -> 左カーブ中 (イン側) -> 直線に戻る (レーン6)."""
    targets = []
    for t in range(300):
        kappa = 0.07 if 60 <= t < 200 else 0.0
        kappa_far = 0.07 if 40 <= t < 180 else 0.0
        st = planner.step(1 / 15, make_lines(offset=-2.9, kappa=kappa, kappa_far=kappa_far), 1.4)
        targets.append(st['target_lane'])
    assert targets[30] == 6
    assert min(targets[80:180]) <= 2
    assert targets[-1] == 6


def test_cone_blocks_lane(planner):
    for _ in range(10):
        st = planner.step(1 / 15, make_lines(offset=-2.9), 1.4)
    assert st['target_lane'] == 6
    lane6_y = core.lane_y(make_lines(offset=-2.9), 5.0, 5.5)
    st = planner.step(1 / 15, make_lines(offset=-2.9), 1.4, cones=[(5.0, lane6_y)])
    assert 6 in st['blocked'] and st['target_lane'] != 6
    # コーンが視野外に消えても, 車体を抜けるまでは塞がれたまま
    for _ in range(10):
        st = planner.step(1 / 15, make_lines(offset=-2.9), 1.4)
    assert 6 in st['blocked']


def test_lost_lines_stop(planner):
    for _ in range(40):
        planner.step(1 / 15, make_lines(offset=-2.9), 1.4)
    v0 = planner.last['v']
    assert v0 > 0.5
    st = planner.step(0.5, None, 1.4)
    assert st['phase'] == core.LOST and st['v'] == pytest.approx(v0)  # 短い欠落は維持
    for _ in range(40):
        st = planner.step(0.1, None, 1.4)
    assert st['v'] == 0.0 and st['target_lane'] == 6


def test_policy_agrees_with_teacher():
    p = core.SixLaneParams()
    net = core.LanePolicyNet.load(POLICY)
    cases = [  # (v, kappas, F) -> 期待するレーン
        (1.4, (0.0, 0.0, 0.0), 3.0, 6),     # 直線: 外側
        (1.4, (0.0, 0.01, 0.08), 5.5, 6),   # 左カーブ手前: アウト
        (1.4, (0.08, 0.08, 0.08), 5.5, 1),  # 左カーブ中: イン
        (1.4, (0.08, 0.03, 0.0), 1.5, 6),   # 左カーブ出口: アウト
    ]
    for v, kap, F, lane in cases:
        probs, _ = net.forward(core.features(v, kap, F, 1.0, p))
        assert int(np.argmax(probs)) + 1 == lane, (v, kap, F, probs)
        assert int(np.argmax(core.teacher_distribution(v, kap, F, 1.0, p))) + 1 == lane


def test_lateral_glitch_rejected(planner):
    """白線の役割が1フレームで1本ずれても (右白線を中央線と取り違え) 横位置は跳ばず, 続けば観測を信じ直す."""
    for _ in range(20):
        st = planner.step(1 / 15, make_lines(offset=-2.9), 1.2)
    F_true, path_y = st['F'], planner.last['lookahead'][1]
    # 取り違え: 全部の線が 3.5m 右にずれて見える (= 車が中央線の左 0.6m にいるように見える)
    st = planner.step(1 / 15, make_lines(offset=-2.9 + 3.5), 1.2)
    assert st['lateral_rejected'] and st['F_meas'] < 3.0
    assert st['F'] == pytest.approx(F_true, abs=0.05) and st['current_lane'] == 6
    assert st['lookahead'][1] == pytest.approx(path_y, abs=0.05)  # 目標点も変わらない
    for _ in range(int(planner.p.lateral_resync_time * 15) + 1):
        st = planner.step(1 / 15, make_lines(offset=-2.9 + 3.5), 1.2)
    assert not st['lateral_rejected'] and st['F'] == pytest.approx(st['F_meas'])


def test_vehicle_lane_coordinate_with_heading():
    """車が車線に対して斜めでも, 前方で測った横位置を車軸位置に戻せる (直線なら x=0 で直接測った値と一致)."""
    for psi in (-0.3, 0.0, 0.2):
        c1 = math.tan(psi)
        lines = {r: core.LineObs(c=((d + 2.0) / math.cos(psi), c1, 0.0), x_min=1.2, x_max=11.5, detected=True)
                 for r, d in zip(core.ROLES, (3.5, 0.0, -3.5))}
        assert core.vehicle_lane_coordinate(lines, 1.5) == pytest.approx(core.lane_coordinate(lines, 0.0, 0.0), abs=0.02)


def test_reanchored_jump_is_accepted(planner):
    """白線の追跡側が根拠をもって付け直したフレームの跳びは取り違えとみなさない (付け直しを打ち消さない)."""
    for _ in range(20):
        planner.step(1 / 15, make_lines(offset=0.6), 1.2)          # 取り違えた状態: 中央線の少し左にいると思っている
    st = planner.step(1 / 15, make_lines(offset=-2.9), 1.2, reanchored=True)   # 付け直し -> 実は右端
    assert not st['lateral_rejected'] and st['current_lane'] == 6
    assert st['F'] == pytest.approx(st['F_meas'])


def test_explain_ja_matches_simulator_text():
    """判断の説明 (実機の status 'explain' / 判断パネル) が web_simulator の explainJa() と同じ文面になる.
    期待値は同じ状態を six_lane_planner.js の explainJa() に入れた出力."""
    st = {'phase': 'APEX', 'sign': 1, 'teacher_target': 2.768311858567773, 'intensity': 0.555668007976305,
          'current_lane': 3, 'target_lane': 1, 'pending_lane': 3, 'pending_count': 2,
          'kappas': [0.03791818449401534, 0.04313537415631128, 0.046101998035306326], 'confidence': 2 / 3,
          'nn_probs': [0.0458, 0.3371, 0.5308, 0.0617, 0.0085, 0.0161], 'blocked': [], 'v': 1.3982620035341973,
          'lateral_rejected': False, 'F_meas': 2.38, 'lost_time': 0}
    assert core.explain_ja(st, core.SixLaneParams()) == [
        '速度 1.40 m/s ／ 曲率 近+0.038 中+0.043 遠+0.046 [1/m]',
        '局面: カーブ旋回中　左カーブ旋回中 (強さ56%) → イン側へ切り込む (目安レーン2.8)',
        'NN推奨 レーン3 (53%) → レーン3へ切替待ち 2/7',
        '白線検出の信頼度 67% (3本中2本)',
    ]
    assert core.explain_ja({'phase': core.LOST, 'lost_time': 1.2}, core.SixLaneParams()) == [
        '白線を見失っています (1.2s)', '→ 減速して停止します']


def test_decision_panel_renders(planner):
    """判断パネル画像 (RViz 用) が白線・コーン・信号つきで描ける (日本語フォントの有無によらず)."""
    import importlib.util
    path = os.path.join(os.path.dirname(__file__), '..', 'oit_navigation', 'utils', 'debug_panel.py')
    spec = importlib.util.spec_from_file_location('debug_panel', path)
    dp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(dp)
    lines = make_lines(offset=-2.9)
    for _ in range(10):
        st = planner.step(1 / 15, lines, 1.2, [(4.0, -0.5)])
    tl = dp.traffic_light_summary({'state': 'APPROACH', 'distance': 9.0, 'stop_distance': 7.0, 'reason': '減速'}, 9.0, None)
    for font in ('', '/nonexistent.ttf'):
        jt = dp.JapaneseText(font)
        if font:
            jt.path = None   # フォントなしの経路も通す
        img = dp.six_lane_panel(jt, lines, st, core.explain_ja(st, planner.p), core.lane_y, cones=[(4.0, -0.5)], tl=tl)
        assert img.ndim == 3 and img.shape[2] == 3 and img.mean() > 10
        img = dp.text_panel(jt, '周回マップ + QP 走行の判断', ['状態: 1周目', ('信号: 減速', dp.YELLOW)])
        assert img.shape[:2] == (260, 620)
