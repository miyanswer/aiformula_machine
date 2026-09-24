"""lane_nav (UFLD 白線ベース周回マップ + QP レーシングライン) のユニットテスト."""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from oit_navigation.lane_nav import (  # noqa: E402
    CameraModel, LineTracker, LineTrackerParams, RecorderParams, RacelineParams, RACING,
    fit_line, ground_to_image, optimize_raceline, project_to_ground, spacing_for_curvature,
)
from oit_navigation.lane_nav.course_map import LapDetectorParams, build_course_map  # noqa: E402
from oit_navigation.lane_nav.boundary_recorder import BoundarySample  # noqa: E402


def test_ground_projection_roundtrip():
    cam = CameraModel(pitch_down=math.radians(7.3))
    # 地面点 -> 画像 -> 地面 が一致すること
    X, Y = np.array([2.0, 5.0, 8.0]), np.array([1.0, -1.5, 0.3])
    fx, fy, cx, cy = cam.scaled(640, 360)
    xr, zr = X - cam.cam_x, -cam.cam_height          # カメラ中心基準 (前方, 上)
    s, c = math.sin(cam.pitch_down), math.cos(cam.pitch_down)
    zc = c * xr - s * zr                              # 光軸方向
    yc = -(s * xr + c * zr)                           # 画像下向き
    u = cx + fx * (-Y) / zc
    v = cy + fy * yc / zc
    x, y, valid = project_to_ground(cam, u, v, 640, 360)
    assert valid.all()
    np.testing.assert_allclose(x, X, atol=1e-6)
    np.testing.assert_allclose(y, Y, atol=1e-6)
    u2, v2, ok = ground_to_image(cam, X, Y, 640, 360)
    assert ok.all()
    np.testing.assert_allclose(u2, u, atol=1e-6)
    np.testing.assert_allclose(v2, v, atol=1e-6)


def test_horizon_points_invalid():
    cam = CameraModel()
    _, _, valid = project_to_ground(cam, np.array([320.0]), np.array([100.0]), 640, 360)
    assert not valid[0]


def _line(offset, curv=0.0):
    xs = np.linspace(1.5, 8.0, 15)
    return fit_line(xs, offset + curv * xs ** 2)


def test_tracker_assigns_by_position_not_order():
    tr = LineTracker(LineTrackerParams(lane_width_init=1.75))
    out = tr.update([_line(-1.7), _line(0.05), _line(1.8)][::-1])
    assert all(out.detected.values())
    assert out.offsets["left"] > out.offsets["center"] > out.offsets["right"]
    assert abs(out.offsets["center"] - 0.05) < 0.05


def test_tracker_fills_missing_lines_from_width():
    tr = LineTracker(LineTrackerParams(lane_width_init=1.75))
    tr.update([_line(-1.7), _line(0.0), _line(1.7)])
    out = tr.update([_line(0.1)])            # 中央線だけ見えた
    assert out.detected["center"] and not out.detected["left"]
    assert out.lines["left"].inferred
    assert abs(out.offsets["left"] - (0.1 + tr.lane_w["left"])) < 1e-6
    out = tr.update([_line(-1.65)])          # 右境界だけ見えた -> 中央線を補完
    assert out.detected["right"] and not out.detected["center"]
    assert out.lines["center"] is not None


def test_tracker_rejects_branch_line():
    """合流部の分岐線 (道幅と合わない間隔 / 向きが違う) を境界として採用しない."""
    tr = LineTracker(LineTrackerParams(lane_width_init=3.0))
    tr.update([_line(3.0), _line(0.0), _line(-3.0)])
    xs = np.linspace(1.5, 8.0, 15)
    branch = fit_line(xs, 1.9 + 0.5 * xs)     # 左境界の予測 (3.0) に近いが斜めに離れていく線
    out = tr.update([branch, _line(0.05), _line(-2.95)])
    assert not out.detected["left"]
    assert out.detected["center"] and out.detected["right"]
    assert abs(out.offsets["left"] - (0.05 + tr.lane_w["left"])) < 1e-6


def test_tracker_prefers_inner_line_of_double_boundary():
    """外側の二重線 (境界 -3.1 と路肩 -3.9) では内側を右境界にする (予測が外側に寄っていても)."""
    tr = LineTracker(LineTrackerParams(lane_width_init=3.1))
    tr.offsets["right"] = -3.9
    tr.lane_w["right"] = 3.9
    out = tr.update([_line(3.1), _line(0.0), _line(-3.1), _line(-3.9)])
    assert abs(out.offsets["right"] + 3.1) < 0.05


def _lane6_tracker():
    """右端 (中央線から 2.9m 右) を走っている状態を追跡済みの tracker. 左白線は視野外."""
    tr = LineTracker(LineTrackerParams(lane_width_init=3.5, init_offset=-2.9))
    for _ in range(3):
        tr.update([_line(2.9), _line(-0.6)])
    return tr


def test_tracker_keeps_lateral_position_when_lost():
    """見失い続けても「中央線の上」に戻さない: 右に寄った状態で右白線だけ再び見えたら右白線のまま."""
    tr = _lane6_tracker()
    for _ in range(LineTrackerParams().lost_reset_frames + 5):
        tr.update([])
    out = tr.update([_line(-0.6)])
    assert out.detected["right"] and not out.detected["center"]
    assert abs(out.offsets["center"] - 2.9) < 0.05


def test_tracker_init_offset():
    """起動時の横位置を指定できる (右端から発進: 右白線を中央線と取り違えない)."""
    tr = LineTracker(LineTrackerParams(lane_width_init=3.5, init_offset=-2.9))
    out = tr.update([_line(-0.6)])
    assert out.detected["right"] and not out.detected["center"]
    tr.reset()
    assert abs(tr.offsets["center"] - 2.9) < 1e-9


def test_tracker_reanchors_when_three_lines_line_up():
    """割り当てが1本ずれていても, 3本が道幅どおりに揃って見え続けたら正しく付け直す."""
    tr = LineTracker(LineTrackerParams(lane_width_init=3.5))
    tr.offsets = {"left": 0.6, "center": -2.9, "right": -6.4}   # 右白線を中央線と取り違えた状態
    lines = [_line(4.1), _line(0.6), _line(-2.9)]                # 実際: 中央線の 0.6m 右を走行
    out = tr.update(lines)
    assert abs(out.offsets["center"] + 2.9) < 0.05              # 1 フレームでは付け直さない
    for _ in range(LineTrackerParams().anchor_frames):
        out = tr.update(lines)
    assert all(out.detected.values())
    assert abs(out.offsets["center"] - 0.6) < 0.05


def test_tracker_reanchor_ignores_double_line_stripe():
    """外側二重線の外側の線は道幅と合わないので再アンカーの根拠にしない."""
    tr = LineTracker(LineTrackerParams(lane_width_init=3.1))
    for _ in range(5):
        out = tr.update([_line(3.1), _line(0.0), _line(-3.1), _line(-3.9)])
    assert abs(out.offsets["center"]) < 0.05 and abs(out.offsets["right"] + 3.1) < 0.05


def test_tracker_double_line_marks_boundary():
    """中央線の上と思い込んで右端から発進しても, 右白線が二重線なら右境界と分かって付け直す."""
    tr = LineTracker(LineTrackerParams(lane_width_init=3.5))      # 初期仮定: 中央線の上
    lines = [_line(2.9), _line(-0.6), _line(-1.3)]                # 中央線 / 右境界 (二重線の内側, 外側)
    out = tr.update(lines)
    assert abs(out.offsets["center"] + 0.6) < 0.05                # 最初は右白線を中央線と取り違える
    for _ in range(LineTrackerParams().anchor_frames):
        out = tr.update(lines)
    assert abs(out.offsets["center"] - 2.9) < 0.05
    assert abs(out.offsets["right"] + 0.6) < 0.05                 # 境界は二重線の内側 (車に近い方)


def test_tracker_reanchor_ignores_far_extrapolated_line():
    """遠方 (9.8m より先) にしか見えない斜めの線を外挿した位置は, 付け直しの根拠にしない (実測の誤作動)."""
    tr = LineTracker(LineTrackerParams(lane_width_init=3.5, init_offset=-2.9))
    xs = np.linspace(9.8, 12.0, 10)
    far = fit_line(xs, 9.68 - 0.298 * xs)                        # 向きの差 0.29rad, x_ref=2 へ外挿すると約 +9.1
    for _ in range(6):
        out = tr.update([far, _line(-0.7), _line(2.73)])
    assert abs(out.offsets["center"] - 2.73) < 0.05 and abs(out.offsets["right"] + 0.7) < 0.05


def test_tracker_seed_lane_position():
    """6レーン座標 (左白線=0, 中央線=3, 右白線=6) の車両位置から線の位置を置き直せる."""
    tr = LineTracker(LineTrackerParams(lane_width_init=3.5))
    tr.seed_lane_position(5.5)                                   # 中央線から 2.92m 右
    assert abs(tr.offsets["center"] - 2.5 * 3.5 / 3) < 1e-9
    assert abs(tr.offsets["right"] - (2.5 * 3.5 / 3 - 3.5)) < 1e-9
    out = tr.update([_line(-0.58)])
    assert out.detected["right"]


def test_spacing_dense_in_curves():
    p = RecorderParams()
    assert spacing_for_curvature(0.0, p) == p.ds_straight
    assert spacing_for_curvature(0.5, p) == p.ds_curve
    assert p.ds_curve < spacing_for_curvature(0.1, p) < p.ds_straight


def _circle_sections(n=80, r_center=10.0, half=1.75):
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    c = np.stack([np.cos(th), np.sin(th)], 1)
    return c * (r_center + half), c * (r_center - half)   # 反時計回り: 左 = 内側


def test_qp_bounds_and_constant_on_circle():
    outer, inner = _circle_sections()
    left, right = inner, outer          # 反時計回りなので左が内側
    p = RacelineParams()
    rl = optimize_raceline(left, right, p)
    width = np.linalg.norm(right - left, axis=1)
    m = (p.vehicle_half_width + p.safety_margin) / width
    assert (rl.alpha >= m - 1e-6).all() and (rl.alpha <= 1 - m + 1e-6).all()
    # 円では最小曲率 = できるだけ外側 (半径最大) を一定に回る
    assert np.ptp(rl.alpha) < 0.05


def test_qp_out_in_out_on_corner():
    """直線 -> 90 度コーナー -> 直線 で, コーナー頂点付近は内側・前後は外側寄り."""
    import lane_nav_sim as sim
    track = sim.make_track()
    idx = np.arange(0, len(track["center"]), 15)
    rl = optimize_raceline(track["left"][idx], track["right"][idx], RacelineParams())
    # 右上 R=6 のコーナー (中心 (32, 18)) の頂点: 左 (内側) 寄り = alpha 小
    apex = np.argmin(np.hypot(rl.points[:, 0] - (32 + 6 * math.cos(math.pi / 4)),
                              rl.points[:, 1] - (18 + 6 * math.sin(math.pi / 4))))
    assert rl.alpha[apex] < 0.35
    # コーナー手前の右直線中央 (38, 13) 付近は外側 (右) 寄り
    before = np.argmin(np.hypot(rl.points[:, 0] - 38, rl.points[:, 1] - 13))
    assert rl.alpha[before] > 0.55


def test_qp_matches_scipy_reference():
    scipy_opt = pytest.importorskip("scipy.optimize")
    from oit_navigation.lane_nav.raceline_qp import solve_box_lsq
    rng = np.random.default_rng(0)
    A = rng.normal(size=(40, 15))
    b = rng.normal(size=40)
    lo, hi = np.full(15, -0.2), np.full(15, 0.3)
    ours = solve_box_lsq(A, b, lo, hi, np.zeros(15), iters=20000, tol=1e-12)
    ref = scipy_opt.lsq_linear(A, b, bounds=(lo, hi)).x
    np.testing.assert_allclose(ours, ref, atol=1e-5)


def test_loop_closure_removes_lateral_drift_only():
    # 真の円周上の断面に, 走行距離に比例した横ずれ (外向き) を足したものを記録として与える
    n = 60
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    samples = []
    for i, t in enumerate(th):
        drift = 0.8 * i / n
        c = np.array([math.cos(t), math.sin(t)])
        samples.append(BoundarySample(
            s=10.0 * t, pose=(0, 0, 0),
            left=tuple(c * (10 - 1.75) + c * drift), right=tuple(c * (10 + 1.75) + c * drift),
            kappa=0.1, left_detected=True, right_detected=True))
    m = build_course_map(samples, (10, 0, math.pi / 2), LapDetectorParams())
    r = np.linalg.norm(m.center, axis=1)
    assert abs(r[-1] - 10.0) < 0.15      # 補正前は 10.8
    assert 0.6 < m.closure_error < 1.0


@pytest.mark.parametrize("bias,yaw_corr", [
    (0.0, False), (0.0003, False), (-0.0003, False),   # 停止中のバイアス推定後に残る程度のバイアス (既定: 方位補正なし)
    (0.001, True), (-0.001, True),                     # 大きなバイアスは方位ドリフト補正を有効にして吸収
])
def test_offline_sim_laps(bias, yaw_corr):
    import lane_nav_sim as sim
    from oit_navigation.lane_nav import NavigatorParams
    params = NavigatorParams()
    params.lap.yaw_drift_correction = yaw_corr
    nav, track, log, _ = sim.run(260.0, yaw_rate_bias=bias, speed_scale=1.01, params=params)
    assert nav.state == RACING and nav.lap >= 4
    st = np.array(log["state"])
    err = sim.lateral_error_to_center(track, log["true"][:, :2])
    racing = st == RACING
    # 車体中心がコース境界 (1.75m) より十分内側
    assert err[racing].max() < 1.55
    # 1 周目は中央線の上
    assert err[~racing].max() < 0.3
    # 2 周目以降はアウト・イン・アウト (中央線から横に振れている)
    assert err[racing].mean() > 0.4


def test_mask_lines_extracts_three_lines_from_synthetic_mask():
    """地面の 3 本線を画像に描いたマスクから 3 本の点列が取れ, 投影すると元の横位置に戻る."""
    import cv2
    from oit_navigation.lane_nav.mask_lines import extract_mask_lines
    cam = CameraModel()
    w, h = 640, 360
    mask = np.zeros((h, w), np.uint8)
    xs = np.linspace(1.2, 15, 200)
    for off in (3.0, 0.0, -3.0):
        u, v, ok = ground_to_image(cam, xs, np.full_like(xs, off) + 0.01 * xs ** 2, w, h)
        pts = np.stack([u[ok], v[ok]], 1).astype(np.int32)
        cv2.polylines(mask, [pts], False, 1, 3)
    lanes = extract_mask_lines(mask)
    assert len(lanes) == 3
    offsets = []
    for l in lanes:
        x, y, valid = project_to_ground(cam, l["u"], l["v"], w, h)
        f = fit_line(x[valid], y[valid])
        offsets.append(f.y_at(2.0))
    np.testing.assert_allclose(sorted(offsets), [-2.96, 0.04, 3.04], atol=0.12)


def test_async_optimization_and_map_json_roundtrip():
    from oit_navigation.lane_nav import CourseMap, LaneNavigator, NavigatorParams, OPTIMIZING
    n = 60
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    samples = []
    for t in th:
        c = np.array([math.cos(t), math.sin(t)])
        samples.append(BoundarySample(s=10.0 * t, pose=(0, 0, 0), left=tuple(c * 8.25), right=tuple(c * 11.75),
                                      kappa=0.1, left_detected=True, right_detected=True))
    nav = LaneNavigator(NavigatorParams(), async_optimize=True)
    nav.recorder.samples = samples
    nav.start_pose = (10.0, 0.0, math.pi / 2)
    assert nav.finish_mapping(nav.start_pose, 63.0)
    assert nav.state == OPTIMIZING
    cmap = nav.pending_course_map()
    rl = optimize_raceline(cmap.left, cmap.right, nav.p.raceline)
    nav.finish_optimization(rl)
    assert nav.state == RACING and len(nav.raceline.points) == len(cmap.left)

    loaded = CourseMap.from_json(cmap.to_json())
    np.testing.assert_allclose(loaded.left, cmap.left)
    nav2 = LaneNavigator(NavigatorParams())
    nav2.load_map(loaded)
    assert nav2.state == RACING


def _straight_center_lines(pose):
    """世界座標 y=0 のまっすぐな中央線 (と ±1.75m の境界線) を, 車体座標のフィットで返す."""
    from oit_navigation.lane_nav.geometry import LineFit
    from oit_navigation.lane_nav.line_tracker import TrackedLines
    x, y, yaw = pose
    # 車体座標で y_body(x_b) ≈ -(y + x_b * tan(yaw)) (小角度の直線)
    c0, c1 = -y / math.cos(yaw), -math.tan(yaw)
    fits = {r: LineFit(np.array([c0 + off, c1, 0.0]), 0.8, 8.0, 20)
            for r, off in (("left", 1.75), ("center", 0.0), ("right", -1.75))}
    return TrackedLines(lines=fits, detected={r: True for r in fits})


def test_mapping_keeps_following_remembered_center_line_through_short_loss():
    """1 周目: 中央線が途切れても即停止→再加速を繰り返さず, 記憶した線をオドメトリで追って
    lines_lost_speed 以下で走り続け, lines_hold_distance を超えて見えなければ停止する."""
    from oit_navigation.lane_nav import LaneNavigator, NavigatorParams
    p = NavigatorParams()
    nav = LaneNavigator(p)
    pose = [0.0, 0.0, 0.0]
    s = t = 0.0
    dt = 1.0 / 15.0
    v = w = 0.0
    log = []
    while t < 20.0:
        visible = t < 4.0                       # 4 秒後から白線が一切見えない
        lines = _straight_center_lines(pose) if visible else None
        cmd = nav.step(t, dt, tuple(pose), v, w, s, lines)
        v, w = cmd.v, cmd.omega
        pose[0] += v * math.cos(pose[2]) * dt
        pose[1] += v * math.sin(pose[2]) * dt
        pose[2] += w * dt
        if t < 4.0 + dt and t >= 4.0 - dt:
            pose[1] += 0.3                      # 見えない間に横ずれ (オドメトリ上) を入れる
        s += v * dt
        t += dt
        log.append((t, v, pose[1], s, nav.message))
    s_lost = next(r[3] for r in log if r[0] > 4.0)
    during = [r for r in log if 4.0 + p.lines_timeout + 0.2 < r[0] and r[3] < s_lost + p.lines_hold_distance - 0.3]
    assert during, "ロスト後に記憶した線で走る区間がある"
    # 止まらずに走り続ける (lines_lost_speed まで減速するだけ)
    assert min(r[1] for r in during) > 0.8 * p.lines_lost_speed
    assert max(r[1] for r in during if r[0] > 4.0 + p.lines_timeout + 1.0) <= p.lines_lost_speed + 1e-6
    assert all(r[4] == "白線ロスト: 記憶した中央線で走行" for r in during)
    # 記憶した線 (y=0) へ戻る: オドメトリ上の横ずれ 0.3m が縮む
    assert abs(during[-1][2]) < 0.2
    # 保持距離を過ぎたら減速停止
    assert log[-1][1] == 0.0 and log[-1][4] == "白線ロスト: 減速停止"
    assert log[-1][3] < s_lost + p.lines_hold_distance + 1.0
