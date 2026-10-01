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


def test_status_has_nn_features_and_measured_speed(planner):
    """status に NN の入力 (features) と実車速 (v_meas) が入り, features で NN を回すと nn_probs が再現できる
    (rosbag から判断を 1 フレームずつ追える). 見失い中も v_meas は更新される."""
    st = planner.step(1 / 15, make_lines(offset=-2.9, kappa=0.03), 1.3)
    assert st['v_meas'] == 1.3 and len(st['features']) == 6
    assert st['features'][0] == pytest.approx(1.3 / planner.p.v_max)
    probs, _ = planner.net.forward(np.array(st['features']))
    assert probs == pytest.approx(st['nn_probs'])
    assert planner.step(0.1, None, 0.9)['v_meas'] == 0.9


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


# ---------------------------------------------------------------------------
# 速度上限の手動切替と自動補正
# ---------------------------------------------------------------------------
def test_speed_limit_is_clamped(planner):
    assert planner.set_speed_limit(10.0) == core.SPEED_LIMIT_MAX
    assert planner.set_speed_limit(0.0) == core.SPEED_LIMIT_MIN
    assert planner.set_speed_limit(2.0) == 2.0


def test_control_scales_with_current_speed_not_with_limit(planner):
    """前方注視点・見失い判定・コーン回避の倍率は, 速度上限ではなく現在の車速で決まる."""
    for lim in (1.5, 3.0):
        planner.set_speed_limit(lim)
        e = core.effective_control(planner.p, 1.5)           # 基準の車速: 補正なし (上限に関係しない)
        assert e['scale'] == 1.0 and e['lookahead_min'] == planner.p.lookahead_min and e['lost_timeout'] == planner.p.lost_timeout
    e = core.effective_control(planner.p, 3.0)
    assert e['scale'] == pytest.approx(2.0)
    assert e['lookahead_min'] == pytest.approx(2.0 * planner.p.lookahead_min)  # 注視点は車速に比例
    assert e['lookahead_max'] == pytest.approx(2.0 * planner.p.lookahead_max)
    assert e['lost_timeout'] == pytest.approx(0.5 * planner.p.lost_timeout)    # 見失い判定は車速に反比例
    e = core.effective_control(planner.p, 0.0)
    assert e['scale'] == pytest.approx(0.5)                                     # 0.5〜2.0 に丸める
    assert e['lost_timeout'] == planner.p.lost_timeout                          # 遅いときは短くしない
    assert core.effective_control(planner.p, 99.0)['scale'] == pytest.approx(2.0)


def test_auto_scale_can_be_disabled(planner):
    planner.p.auto_scale = False
    assert core.effective_control(planner.p, 3.0)['scale'] == 1.0


def test_lookahead_follows_current_speed(planner):
    """同じ速度上限でも, 今の車速が速いほど注視点が遠い (status の lookahead = [tx, ty])."""
    lines = make_lines(offset=0.0, kappa=0.0)
    far = {}
    for v in (0.75, 1.5, 3.0):
        pl = core.SixLanePlanner(core.LanePolicyNet.load(POLICY), core.SixLaneParams())
        pl.set_speed_limit(3.0)                              # 上限は同じ 3.0
        for _ in range(30):
            st = pl.step(1 / 15, lines, v)
        far[v] = st['lookahead'][0]
    assert far[0.75] < far[1.5] < far[3.0]
    assert far[3.0] == pytest.approx(4.5, rel=0.05)         # clamp(3.0 * 1.5 s, 4.0, 7.0)


def test_nn_decision_is_independent_of_speed_limit():
    """NN の速度入力は v / v_max (相対速度) なので, 上限を変えても同じ相対速度なら同じ判断になる."""
    net = core.LanePolicyNet.load(POLICY)
    p1, p2 = core.SixLaneParams(), core.SixLaneParams()
    p2.v_max = 3.0
    k = [0.0, 0.02, 0.06]
    assert np.allclose(core.features(0.75, k, 3.0, 1.0, p1), core.features(1.5, k, 3.0, 1.0, p2))
    assert np.allclose(net.forward(core.features(0.75, k, 3.0, 1.0, p1))[0], net.forward(core.features(1.5, k, 3.0, 1.0, p2))[0])


def test_status_reports_speed_limit(planner):
    planner.set_speed_limit(2.0)
    st = planner.step(1 / 15, make_lines(), 3.0)
    assert st['speed_limit'] == 2.0 and st['speed_scale'] == pytest.approx(2.0)   # 倍率は現在の車速 3.0 / 基準 1.5
    assert any('現在の車速に合わせて自動補正' in line and '速度上限 2.0 m/s' in line for line in core.explain_ja(st, planner.p))


def test_reactive_avoider_scales_with_speed_limit():
    from oit_navigation.lane_core.cone_avoidance import ReactiveAvoider
    cone = [(2.5, 0.2)]  # 2.5 m 先: 基準 (REACT_SLOW_X=1.8 m) では減速しない
    v1, _ = ReactiveAvoider().step(0.0, 0.05, 3.0, 0.0, cone, scale=1.0)
    v2, _ = ReactiveAvoider().step(0.0, 0.05, 3.0, 0.0, cone, scale=2.0)  # 上限 3.0 m/s: 3.6 m 手前から減速
    assert v1 == 3.0 and v2 < 3.0


# ---------------------------------------------------------------------------
# 観測の遅れ補償
# ---------------------------------------------------------------------------
def test_compensate_points_translates_by_travelled_distance():
    out = core.compensate_points([(5.0, 1.0)], v=2.0, omega=0.0, age=0.1)
    assert out[0] == pytest.approx((4.8, 1.0))
    assert core.compensate_points([(5.0, 1.0)], 2.0, 0.0, 0.0) == [(5.0, 1.0)]       # 遅れ 0 は変えない
    assert core.compensate_points([(5.0, 1.0)], 2.0, 0.0, 0.6) == [(5.0, 1.0)]       # latency_max 以上は変えない


def test_compensate_points_rotates_for_yaw_rate():
    # 左旋回 (omega>0) 中は, 前方の点が車から見て右 (y 減少) にずれる
    (x, y), = core.compensate_points([(5.0, 0.0)], v=0.0, omega=1.0, age=0.2)
    assert y < 0 and x == pytest.approx(5.0 * math.cos(0.2), abs=1e-6)


def test_compensate_lines_matches_pointwise_motion():
    lines = make_lines(offset=0.3, kappa=0.04)
    v, om, age = 2.0, 0.3, 0.12
    out = core.compensate_lines(lines, v, om, age)
    for role in core.ROLES:
        old, new = lines[role], out[role]
        # 古い曲線上の点を同じ運動で変換した点が, 新しい 2 次式の上に載る
        for x in (2.0, 5.0, 9.0):
            (xn, yn), = core.compensate_points([(x, old.y_at(x))], v, om, age)
            if new.x_min <= xn <= new.x_max:
                assert new.y_at(xn) == pytest.approx(yn, abs=0.02)
        assert new.x_max < old.x_max  # 進んだ分だけ視野が手前に寄る
        assert len(new.px) == len(old.px)


def test_compensation_keeps_lane_coordinate_consistent():
    """一定速度で直進中, 遅れ補償しても横位置 (レーン座標 F) は変わらない (前方へ進むだけ)."""
    lines = make_lines(offset=0.5, kappa=0.0)
    out = core.compensate_lines(lines, 2.0, 0.0, 0.15)
    assert core.vehicle_lane_coordinate(out, 1.5) == pytest.approx(core.vehicle_lane_coordinate(lines, 1.5), abs=1e-3)


def test_latency_gate_drops_old_and_detects_clock_skew():
    g = core.LatencyGate(latency_max=0.5, skew_frames=3)
    assert g.check(0.1) == (pytest.approx(0.1), True)
    assert g.check(None) == (0.0, True)
    assert g.check(0.9) == (0.0, False)           # 古い観測は捨てる
    assert g.check(0.9) == (0.0, False)
    assert g.check(0.9) == (0.0, True) and g.skewed  # 3 回続いたら時計のずれ: 補償なしで使う
    assert g.check(-5.0) == (0.0, True)
    assert g.check(0.05) == (pytest.approx(0.05), True) and not g.skewed  # 範囲内に戻れば再開


# ---------------------------------------------------------------------------
# 車速推定 (車輪速 + IMU)
# ---------------------------------------------------------------------------
def test_speed_estimator_rejects_wheel_slip_transient():
    est = core.SpeedEstimator(tau=1.0)
    dt = 0.02
    for _ in range(100):               # 1 m/s で巡航
        est.update(1.0, 0.0, dt)
    assert est.v == pytest.approx(1.0, abs=1e-6)
    for _ in range(5):                 # 車輪が一瞬空転 (+0.3 m/s) するが IMU の加速度は 0
        est.update(1.3, 0.0, dt)
    assert est.v < 1.03                # 車輪速だけなら 1.3 だが, 推定はほとんど動かない


def test_speed_estimator_follows_true_acceleration():
    est = core.SpeedEstimator(tau=1.0)
    dt, v = 0.02, 0.0
    for i in range(100):               # 1 m/s^2 で 2 s 加速. 車輪速は +8% のスリップ
        v += 1.0 * dt
        est.update(v * 1.08, 1.0, dt)
    assert abs(est.v - v) < abs(v * 1.08 - v)  # 車輪速そのままより真値に近い
    assert est.imu_ok


def test_speed_estimator_falls_back_to_wheel_speed():
    est = core.SpeedEstimator(tau=1.0)
    assert est.update(1.0, None, 0.02) == 1.0 and not est.imu_ok      # IMU なし
    est2 = core.SpeedEstimator(tau=1.0)
    for _ in range(50):                                                # IMU の向きが逆 (加速しているのに負): 誤差は max_dev 以内に収まる
        est2.update(1.0, -3.0, 0.02)
    assert abs(est2.v - 1.0) <= est2.max_dev
    est3 = core.SpeedEstimator(tau=1.0)                                # 車輪速が急に大きく食い違ったら車輪速をそのまま使う
    assert est3.update(2.0, 0.0, 0.02) == 2.0 and not est3.imu_ok


def test_speed_estimator_zero_velocity_update_and_scale():
    est = core.SpeedEstimator(tau=1.0, scale=0.9)
    for _ in range(200):
        est.update(0.0, 0.2, 0.02)     # 停止中の加速度の偏り 0.2 を推定
    assert est.v == 0.0 and est.bias == pytest.approx(0.2, abs=0.01)
    assert core.SpeedEstimator(scale=0.9).update(1.0, None, 0.02) == pytest.approx(0.9)  # 校正係数


# ---------------------------------------------------------------------------
# 速度上限を変えても走れる (閉ループ): 直線コースで上限へ収束し, 上限を下げると減速する
# ---------------------------------------------------------------------------
def test_closed_loop_follows_speed_limit_changes(planner):
    lines = make_lines(offset=0.0, kappa=0.0)
    planner.set_speed_limit(3.0)
    v = 0.0
    for _ in range(600):
        st = planner.step(1 / 15, lines, v)
        v += max(-1.5 / 15, min(2.2 / 15, st['v'] - v))   # motor_controller の加減速制限
    assert 2.0 < v <= 3.0
    planner.set_speed_limit(1.0)
    for _ in range(300):
        st = planner.step(1 / 15, lines, v)
        v += max(-1.5 / 15, min(2.2 / 15, st['v'] - v))
    assert v == pytest.approx(1.0, abs=0.35)
