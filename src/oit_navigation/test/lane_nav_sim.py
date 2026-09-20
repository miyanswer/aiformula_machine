"""
lane_nav のオフライン 2D シミュレーション (ROS もカメラも使わない検証用ハーネス).

合成コース (中央白線 + 左右境界の 3 本線) の上で差動二輪の車両を動かし,
    - 白線観測: 車両前方の各線の点を車両座標に変換し, ノイズ付きで LineFit にする.
      UFLD の実挙動 (自車付近の 2 本しか出ないことが多い) を真似て, 線をランダムに欠落させる.
    - オドメトリ: 速度・ヨーレートにノイズ/バイアスを入れて積算 (odom_imu_localizer 相当).
を LineTracker -> LaneNavigator に流して 1 周目 (記録) -> 2 周目以降 (QP ライン追従) を回す.

    python3 test/lane_nav_sim.py --plot /tmp/lane_nav_sim.png
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from oit_navigation.lane_nav import (  # noqa: E402
    LaneNavigator, LineTracker, LineTrackerParams, NavigatorParams, RACING, fit_line,
)


def make_track(half_width: float = 1.75, step: float = 0.1):
    """中央線: 直線 + 大小 2 種類のカーブを持つ閉コース (反時計回り). 左右境界は法線方向に ±half_width."""
    pts = []
    # 下の直線 (x: 0 -> 30, y=0)
    for x in np.arange(0, 30, step):
        pts.append((x, 0.0))
    # 右下カーブ R=8 (中心 (30, 8)), -90 -> 0 deg
    for a in np.arange(-math.pi / 2, 0, step / 8):
        pts.append((30 + 8 * math.cos(a), 8 + 8 * math.sin(a)))
    # 右の直線 (y: 8 -> 18)
    for y in np.arange(8, 18, step):
        pts.append((38.0, y))
    # 右上カーブ R=6 (中心 (32, 18)), 0 -> 90
    for a in np.arange(0, math.pi / 2, step / 6):
        pts.append((32 + 6 * math.cos(a), 18 + 6 * math.sin(a)))
    # 上の直線 (x: 32 -> 0, y=24)
    for x in np.arange(32, 0, -step):
        pts.append((x, 24.0))
    # 左の半円 R=12 (中心 (0, 12)), 90 -> 270 deg で (0, 0) に戻る
    for a in np.arange(math.pi / 2, 3 * math.pi / 2, step / 12):
        pts.append((12 * math.cos(a), 12 + 12 * math.sin(a)))
    center = np.array(pts)
    t = np.roll(center, -1, axis=0) - np.roll(center, 1, axis=0)
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    n = np.stack([-t[:, 1], t[:, 0]], axis=1)   # 進行方向左
    return {"center": center, "left": center + half_width * n, "right": center - half_width * n}


def observe(track, pose, rng, drop_prob=(0.35, 0.05, 0.35), noise=0.03, x_range=(1.2, 9.0)):
    """真の車両姿勢から, 前方に見える各線を車両座標の LineFit にする (欠落・ノイズあり)."""
    x0, y0, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    fits = []
    for k, role in enumerate(("left", "center", "right")):
        if rng.random() < drop_prob[k]:
            continue
        rel = track[role] - np.array([x0, y0])
        vx = c * rel[:, 0] + s * rel[:, 1]
        vy = -s * rel[:, 0] + c * rel[:, 1]
        m = (vx > x_range[0]) & (vx < x_range[1]) & (np.abs(vy) < 6.0)
        if m.sum() < 5:
            continue
        idx = np.nonzero(m)[0][:: max(1, int(m.sum() // 20))]
        f = fit_line(vx[idx], vy[idx] + rng.normal(0, noise * (1 + vx[idx] / 5), len(idx)))
        if f is not None:
            fits.append(f)
    rng.shuffle(fits)  # スロット順は意味を持たない
    return fits


def run(seconds=160.0, dt=0.05, yaw_rate_bias=0.002, speed_scale=1.01, seed=0,
        start_offset=0.1, params=None):
    rng = np.random.default_rng(seed)
    track = make_track()
    params = params or NavigatorParams()
    nav = LaneNavigator(params)
    tracker = LineTracker(LineTrackerParams(lane_width_init=1.75))

    true = np.array([2.0, start_offset, 0.0])      # 真の姿勢 (中央線上 + 少しずれ)
    odom = true.copy()                             # オドメトリ (真値と同じ座標系で開始. 以後ドリフトする)
    odom_to_true_yaw = true[2]
    v, w, s = 0.0, 0.0, 0.0
    log = {"t": [], "true": [], "odom": [], "state": [], "v": [], "lap": []}
    t = 0.0
    while t < seconds:
        fits = observe(track, true, rng)
        lines = tracker.update(fits)
        # 計測: 速度・ヨーレート (スケール誤差 / バイアス付き)
        v_meas = v * speed_scale + rng.normal(0, 0.01)
        w_meas = w + yaw_rate_bias + rng.normal(0, 0.005)
        cmd = nav.step(t, dt, tuple(odom), v_meas, w_meas, s, lines)
        # 真の運動 (一次遅れ付き)
        v += (cmd.v - v) * min(1.0, dt / 0.2)
        w += (cmd.omega - w) * min(1.0, dt / 0.1)
        true[0] += v * math.cos(true[2] + 0.5 * w * dt) * dt
        true[1] += v * math.sin(true[2] + 0.5 * w * dt) * dt
        true[2] += w * dt
        # オドメトリ積算 (odom_imu_localizer と同じ中点積分)
        odom[0] += v_meas * math.cos(odom[2] + 0.5 * w_meas * dt) * dt
        odom[1] += v_meas * math.sin(odom[2] + 0.5 * w_meas * dt) * dt
        odom[2] += w_meas * dt
        s += abs(v_meas) * dt
        t += dt
        log["t"].append(t)
        log["true"].append(true.copy())
        log["odom"].append(odom.copy())
        log["state"].append(nav.state)
        log["v"].append(v)
        log["lap"].append(nav.lap)
    for k in ("true", "odom"):
        log[k] = np.array(log[k])
    return nav, track, log, odom_to_true_yaw


def lateral_error_to_center(track, xy):
    d = np.hypot(track["center"][:, 0][None, :] - xy[:, 0][:, None],
                 track["center"][:, 1][None, :] - xy[:, 1][:, None])
    return d.min(axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", default="")
    ap.add_argument("--seconds", type=float, default=160.0)
    ap.add_argument("--bias", type=float, default=0.002)
    args = ap.parse_args()
    nav, track, log, _ = run(args.seconds, yaw_rate_bias=args.bias)
    st = np.array(log["state"])
    t = np.array(log["t"])
    racing = st == RACING
    print("final state:", nav.state, "lap:", nav.lap, "|", nav.message)
    if nav.course_map is not None:
        print(f"map sections: {len(nav.course_map.left)}  closure_error: {nav.course_map.closure_error:.3f} m")
        rl = nav.raceline
        print(f"raceline length {rl.length:.1f} m, alpha range [{rl.alpha.min():.2f}, {rl.alpha.max():.2f}],"
              f" v range [{rl.speed.min():.2f}, {rl.speed.max():.2f}] m/s")
    if racing.any():
        print(f"mapping lap time: {t[racing.argmax()]:.1f}s")
    err = lateral_error_to_center(track, log["true"][:, :2])
    print(f"max |dist to center| lap1: {err[~racing].max():.2f} m, racing: {err[racing].max() if racing.any() else 0:.2f} m"
          f" (half width 1.75, limit {1.75 - 0.4:.2f})")
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(11, 8))
        for k, col in (("left", "k"), ("right", "k"), ("center", "0.6")):
            ax.plot(*track[k].T, col, lw=1, ls="--" if k == "center" else "-")
        ax.plot(*log["true"][~racing, :2].T, color="tab:blue", lw=1, label="true path lap1 (center tracking)")
        ax.plot(*log["true"][racing, :2].T, color="tab:red", lw=1, label="true path lap2+ (raceline)")
        if nav.course_map is not None:
            m = nav.course_map
            ax.plot(*m.left.T, "g.", ms=4, label="recorded left (odom)")
            ax.plot(*m.right.T, "m.", ms=4, label="recorded right (odom)")
            ax.plot(*nav.raceline.points.T, "o", mfc="none", color="orange", ms=5, label="QP waypoints (odom)")
        ax.set_aspect("equal")
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title(f"lane_nav offline sim: state={nav.state} lap={nav.lap}")
        fig.savefig(args.plot, dpi=110, bbox_inches="tight")
        print("saved", args.plot)


if __name__ == "__main__":
    main()
