"""lane_core (白線の幾何・役割割り当て・マスク抽出) のユニットテスト."""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from oit_navigation.lane_core import (  # noqa: E402
    CameraModel, LineTracker, LineTrackerParams, fit_line, ground_to_image, project_to_ground,
)


def test_ground_projection_roundtrip():
    cam = CameraModel(pitch_down=math.radians(1.8))
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


def test_mask_lines_extracts_three_lines_from_synthetic_mask():
    """地面の 3 本線を画像に描いたマスクから 3 本の点列が取れ, 投影すると元の横位置に戻る."""
    import cv2
    from oit_navigation.lane_core.mask_lines import extract_mask_lines
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
