#!/usr/bin/env python3
"""
png/shihou_cource_unity.png の外周コースを実寸の幾何データに起こし、
web_simulator のコース定義一式を生成する.

    python3 web_simulator/tools/build_course.py

出力:
    js/course_geometry.js        基準パス + 寸法定数 (手で編集しない)
    js/course_lines.js           3 本線の点列 (理想検出モードの入力)
    png/shihou_cource_base.png   外周 3 本線を消した背景テクスチャ

設計の根拠は docs/superpowers/specs/2026-09-21-course-geometry-and-collision-design.md.
"""
import json
import os
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

# --- 寸法 (仕様書 1/2/3 節) ---
LANE_WIDTH_M = 3.5          # 中央線 <-> 境界線
LINE_WIDTH_M = 0.15         # 白線の幅
HARMONICS = 30              # FFT ローパスで残す次数
RESAMPLE_STEP_M = 0.10      # 基準パスの等弧長間隔

HERE = os.path.dirname(os.path.abspath(__file__))
SRC_IMG = os.path.join(HERE, "..", "png", "shihou_cource_unity.png")
OUT_IMG = os.path.join(HERE, "..", "png", "shihou_cource_base.png")
OUT_GEOM = os.path.join(HERE, "..", "js", "course_geometry.js")
OUT_LINES = os.path.join(HERE, "..", "js", "course_lines.js")

# --- 抽出 (extract_course_lines.py から引き継ぎ) ---
N_RAYS = 1440
RAY_CENTER = (495, 406)           # 外周ループの内側にある画像座標
SRC_WIDTH_M = 100.0               # 抽出時の暫定スケール
SRC_POSE = (13.22, 35.41)         # 同上
WHITE_THRESHOLD = 195

# --- 実寸化 (仕様書 1 節) ---
SCALE_K = 1.0680                  # 3.5 / 3.2772 (車線幅の実測プール平均)
COURSE_WIDTH_M = SRC_WIDTH_M * SCALE_K   # 106.80
START_WORLD = (0.0, -1.6)         # スポーン地点. 実寸化しても動かさない

# --- 破線 / 接続口 (仕様書 3 節) ---
DASH_MARK_M = 2.8
DASH_GAP_M = 2.6
INNER_GAP_MIN_M = 1.5
# 内側境界線の「線が存在する」判定のしきい値. 抽出サンプル (レイ) は弧長で
# およそ 1440 本 / 内側境界全長 ~225m = ~0.16m 間隔で並び, 生成した内側線
# (基準パスを LANE_WIDTH_M だけオフセットしたもの) は実サンプルから概ね
# 0.3m 以内に収まる. どちらよりも十分大きく, かつ接続口の最小長 1.5m
# (INNER_GAP_MIN_M) よりは十分小さい 1.0m を「線あり」しきい値にする.
INNER_PRESENCE_TOL_M = 1.0

START_YAW = 0.0   # js/simulator.js SIM_START_POSE: travel heading at the start line
                  # (counter-clockwise around the loop). The reference path is
                  # oriented to match, so arc length grows the way the car drives.

# --- 背景テクスチャ (タスク 3) ---
ERASE_CORRIDOR_M = 0.45           # 消去回廊の基準幅. erase_lines() は隣接レイを
                                   # cv2.line(thickness = 1.5*corridor_px) で結ぶため,
                                   # 実際の法線方向の消去半幅は 0.75*ERASE_CORRIDOR_M
                                   # (既定値では約 0.31m) で、この値そのものではない
ASPHALT_BGR = (156, 150, 156)     # 元 PNG のアスファルト色


def fill_closed(line, n_rays):
    # type: (List[Optional[Sequence[float]]], int) -> np.ndarray
    """None (破線の切れ目 / 抽出のドロップアウト) をレイ角の周期線形補間で埋める."""
    idx = [i for i, p in enumerate(line) if p]
    if len(idx) < 3:
        raise ValueError("too few valid samples to close the loop")
    pts = np.array([line[i] for i in idx], dtype=float)
    theta = np.array(idx, dtype=float) / n_rays * 2 * np.pi
    full = np.arange(n_rays, dtype=float) / n_rays * 2 * np.pi
    out = np.empty((n_rays, 2))
    for c in range(2):
        out[:, c] = np.interp(full, theta, pts[:, c], period=2 * np.pi)
    return out


def lowpass_closed(curve, harmonics):
    # type: (np.ndarray, int) -> np.ndarray
    """閉曲線を FFT ローパスして手描き由来のジッタを落とす."""
    spectrum = np.fft.rfft(curve, axis=0)
    spectrum[harmonics + 1:] = 0
    return np.fft.irfft(spectrum, n=len(curve), axis=0)


def resample_closed(curve, step_m):
    # type: (np.ndarray, float) -> Tuple[np.ndarray, float]
    """閉曲線を等弧長で再サンプルする. 戻り値は (点列, 全長)."""
    seg = np.hypot(*(np.roll(curve, -1, axis=0) - curve).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    count = max(int(round(total / step_m)), 4)
    target = np.arange(count) * (total / count)
    closed = np.vstack([curve, curve[:1]])
    out = np.empty((count, 2))
    for c in range(2):
        out[:, c] = np.interp(target, s, closed[:, c])
    return out, total


def normals_closed(path):
    # type: (np.ndarray) -> np.ndarray
    """各点の単位左手法線 (進行方向を +90 度回した向き)."""
    tangent = np.roll(path, -1, axis=0) - np.roll(path, 1, axis=0)
    length = np.hypot(*tangent.T)
    length[length < 1e-12] = 1e-12
    tangent = tangent / length[:, None]
    return np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)


def offset_closed(path, distance):
    # type: (np.ndarray, float) -> np.ndarray
    """基準パスを法線方向に distance だけオフセットした線."""
    return path + normals_closed(path) * distance


def curvature_closed(path):
    # type: (np.ndarray) -> np.ndarray
    """閉曲線の符号付き曲率 [1/m]."""
    # For a closed curve, use central differences everywhere with wrapping
    # to avoid edge effects from np.gradient's forward/backward difference scheme.
    dx = (np.roll(path, -1, axis=0)[:, 0] - np.roll(path, 1, axis=0)[:, 0]) / 2
    dy = (np.roll(path, -1, axis=0)[:, 1] - np.roll(path, 1, axis=0)[:, 1]) / 2
    ddx = (np.roll(dx, -1) - np.roll(dx, 1)) / 2
    ddy = (np.roll(dy, -1) - np.roll(dy, 1)) / 2
    denom = np.maximum((dx * dx + dy * dy) ** 1.5, 1e-12)
    return (dx * ddy - dy * ddx) / denom


def nearest_distance(points, polyline):
    # type: (np.ndarray, np.ndarray) -> np.ndarray
    """各点から閉じた polyline への最短距離 (線分までの距離)."""
    a = polyline
    b = np.roll(polyline, -1, axis=0)
    ab = b - a
    denom = np.maximum((ab * ab).sum(axis=1), 1e-12)
    out = np.empty(len(points))
    for i, p in enumerate(points):
        t = np.clip(((p - a) * ab).sum(axis=1) / denom, 0.0, 1.0)
        proj = a + ab * t[:, None]
        out[i] = np.hypot(*(proj - p).T).min()
    return out


def extract_ray_radii(white, center, n_rays):
    # type: (np.ndarray, Tuple[int, int], int) -> dict
    """
    画像中心付近から放射状にレイを飛ばし, 外側から内側へ白線の横断位置を拾う.
    1, 2 本目 = 外側の二重線 (2 本目がレーン境界). 以降は 2 本目からの距離で
    中央線 (2.3-3.9m) / 内側境界 (5.0-7.6m) を判定する.
    戻り値はレイ上の半径 [px]. 線が無ければ None.
    """
    h, w = white.shape
    mpp = SRC_WIDTH_M / w
    cx, cy = center
    lines = {"outer": [], "center": [], "inner": []}
    for k in range(n_rays):
        theta = 2 * np.pi * k / n_rays
        dx, dy = np.cos(theta), np.sin(theta)
        rs = np.arange(560, 120, -0.5)
        xs = (cx + rs * dx).round().astype(int)
        ys = (cy + rs * dy).round().astype(int)
        ok = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
        vals = np.zeros(len(rs), bool)
        vals[ok] = white[ys[ok], xs[ok]]
        runs = []
        i = 0
        while i < len(vals):
            if vals[i]:
                j = i
                while j < len(vals) and vals[j]:
                    j += 1
                runs.append(rs[(i + j - 1) // 2])
                i = j
            else:
                i += 1
        o = c = n = None
        if len(runs) >= 2 and runs[0] - runs[1] < 12:
            o = runs[1]
            for r in runs[2:]:
                d = (o - r) * mpp
                if 2.3 < d < 3.9 and c is None:
                    c = r
                elif 5.0 < d < 7.6 and n is None:
                    n = r
        lines["outer"].append(o)
        lines["center"].append(c)
        lines["inner"].append(n)
    return lines


def radii_to_world(radii, center, n_rays, image_shape, width_m, pose):
    # type: (List[Optional[float]], Tuple[int, int], int, Tuple[int, int], float, Tuple[float, float]) -> List[Optional[List[float]]]
    """レイ半径 [px] を ROS 世界座標 [m] に変換する."""
    h, w = image_shape
    depth_m = width_m * h / w
    cx, cy = center
    out = []
    for k, r in enumerate(radii):
        if r is None:
            out.append(None)
            continue
        theta = 2 * np.pi * k / n_rays
        px = cx + r * np.cos(theta)
        py = cy + r * np.sin(theta)
        lx = (px / w - 0.5) * width_m
        ly = (0.5 - py / h) * depth_m
        out.append([pose[0] - ly, pose[1] + lx])
    return out


def gap_intervals(line, seg_lengths, min_gap_m):
    # type: (List[Optional[object]], np.ndarray, float) -> List[Tuple[float, float]]
    """
    line の欠損区間のうち min_gap_m を超えるものを弧長区間 [s0, s1) で返す.
    インデックス 0 を跨ぐ欠損は s1 > 全長 となる 1 区間にまとめる.
    """
    n = len(line)
    s = np.concatenate([[0.0], np.cumsum(seg_lengths)])
    total = float(s[-1])
    if all(p is None for p in line):
        return [(0.0, total)]
    start = next(i for i in range(n) if line[i] is not None)
    out = []
    run_start = None
    for step in range(n + 1):
        i = (start + step) % n
        missing = step < n and line[i] is None
        if missing and run_start is None:
            run_start = start + step
        elif not missing and run_start is not None:
            s0 = float(s[run_start % n]) + total * (run_start // n)
            s1 = float(s[(start + step) % n]) + total * ((start + step) // n)
            if s1 - s0 > min_gap_m:
                out.append((s0, s1))
            run_start = None
    return out


def build_geometry():
    # type: () -> dict
    """PNG から実寸の幾何定義を組み立てる. 生成物には書き込まない."""
    image = cv2.imread(SRC_IMG)
    if image is None:
        raise IOError("cannot read %s" % SRC_IMG)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    white = gray > WHITE_THRESHOLD

    radii = extract_ray_radii(white, RAY_CENTER, N_RAYS)
    traced = {}
    for name in ("outer", "center", "inner"):
        traced[name] = radii_to_world(
            radii[name], RAY_CENTER, N_RAYS, gray.shape, COURSE_WIDTH_M, SRC_POSE
        )

    # 1. 中央線を平滑化して基準パスにする
    filled = fill_closed(traced["center"], N_RAYS)
    smooth = lowpass_closed(filled, HARMONICS)
    path, _ = resample_closed(smooth, RESAMPLE_STEP_M)

    # 2. 世界原点をスポーン地点に合わせ直す.
    #    抽出は SRC_POSE を仮の板位置として行うため, 平滑化後のパスが
    #    START_WORLD を通るように平行移動する. 板の位置 (COURSE_POSE) も
    #    同じ量だけずらすので, 背景 PNG との相対関係は保たれる.
    anchor_idx = int(np.argmin(np.hypot(path[:, 0] - START_WORLD[0], path[:, 1] - START_WORLD[1])))
    shift = np.array(START_WORLD) - path[anchor_idx]
    path = path + shift
    path = np.roll(path, -anchor_idx, axis=0)   # s=0 をスポーン地点にする
    for name in traced:
        traced[name] = [None if p is None else [p[0] + shift[0], p[1] + shift[1]]
                        for p in traced[name]]

    # 2b. 進行方向を js/simulator.js の SIM_START_POSE.yaw (0 = +x, ループを
    #     反時計回りに周回) に合わせる. 抽出順序 (レイ角の増加方向) がコースの
    #     実際の走行方向と一致している保証は無いため, index 0 の接線が
    #     START_YAW から 90 度以上ずれていたら経路を反転する. これで弧長が
    #     走行方向に増えるようになり, 以降の外側/内側判定・接続口の弧長投影・
    #     startPose.yaw が正しい向きで計算される. 反転すると錨点が末尾
    #     (index -1) に移るので, 1 だけロールして index 0 に戻す.
    tangent0 = path[1] - path[-1]
    if np.cos(np.arctan2(tangent0[1], tangent0[0]) - START_YAW) < 0:
        path = np.roll(path[::-1], 1, axis=0)

    seg = np.hypot(*(np.roll(path, -1, axis=0) - path).T)
    total = float(seg.sum())

    # 3. 外側 / 内側がパスのどちら側かを, 抽出線との距離で決める
    plus = offset_closed(path, LANE_WIDTH_M)
    traced_outer = np.array([p for p in traced["outer"] if p])
    sign_outer = 1.0 if nearest_distance(traced_outer, plus).mean() < \
        nearest_distance(traced_outer, offset_closed(path, -LANE_WIDTH_M)).mean() else -1.0

    lines = {
        "outer": offset_closed(path, sign_outer * LANE_WIDTH_M),
        "center": path,
        "inner": offset_closed(path, -sign_outer * LANE_WIDTH_M),
    }

    # 4. 内側境界線の接続口 (実際に線が途切れている区間) を判定する.
    #
    #    生成した内側境界線 (path を -LANE_WIDTH_M だけオフセットしたもの) の
    #    各点について, 抽出できた内側サンプルが近く (INNER_PRESENCE_TOL_M 以内)
    #    に実在するかを path のインデックス空間で直接判定する presence test.
    #    line / seg_lengths の両方を path のインデックスでそろえて渡すので,
    #    gap_intervals が組み立てる弧長軸は path 自身の s = cumsum(seg) と
    #    常に一致する.
    inner_line_pts = np.array(lines["inner"])
    traced_inner_pts = np.array([p for p in traced["inner"] if p is not None])
    d_to_traced = np.array([
        np.hypot(*(traced_inner_pts - p).T).min() for p in inner_line_pts
    ])
    present_line = [True if d < INNER_PRESENCE_TOL_M else None for d in d_to_traced]
    inner_gaps = gap_intervals(present_line, seg, INNER_GAP_MIN_M)

    tangent = path[1] - path[-1]
    start_yaw = float(np.arctan2(tangent[1], tangent[0]))

    return {
        "laneWidthM": LANE_WIDTH_M,
        "lineWidthM": LINE_WIDTH_M,
        "lengthM": round(total, 3),
        "startPose": {"x": round(float(path[0][0]), 4),
                      "y": round(float(path[0][1]), 4),
                      "yaw": round(start_yaw, 5)},
        "dash": {"markM": DASH_MARK_M, "gapM": DASH_GAP_M},
        "outerSign": sign_outer,
        "innerGaps": [[round(a, 3), round(b, 3)] for a, b in inner_gaps],
        # mm 単位 (3 桁) だと丸め誤差だけで車線幅の 1mm 許容 (仕様書 1 節) を
        # 割り込みかねないので, 0.1mm (4 桁) で保持する.
        "centerPath": [[round(float(p[0]), 4), round(float(p[1]), 4)] for p in path],
        "_lines": {k: [[round(float(p[0]), 4), round(float(p[1]), 4)] for p in v]
                   for k, v in lines.items()},
        "_traced": traced,
        "_radii": radii,
        "_shift": [float(shift[0]), float(shift[1])],
    }


def write_geometry_js(geom, path=OUT_GEOM):
    # type: (dict, str) -> None
    public = {k: v for k, v in geom.items() if not k.startswith("_")}
    with open(path, "w") as f:
        f.write("// Generated by web_simulator/tools/build_course.py -- do not edit.\n")
        f.write("// Outer-loop reference path and dimensions, in world (ROS) metres.\n")
        f.write("// centerPath is a closed, %.2f m equal-arc-length polyline; every\n" % RESAMPLE_STEP_M)
        f.write("// white line is generated from it, so the drawn course and the\n")
        f.write("// course the ideal detector sees are the same geometry by construction.\n")
        f.write("export const COURSE_GEOMETRY = ")
        json.dump(public, f, separators=(",", ":"))
        f.write(";\n")


def dashed_line(line, s, mark_m, gap_m):
    # type: (List[List[float]], np.ndarray, float, float) -> List[Optional[List[float]]]
    """破線パターンの「切れ目」を None にする."""
    pitch = mark_m + gap_m
    return [p if (s[i] % pitch) < mark_m else None for i, p in enumerate(line)]


def gapped_line(line, s, total, intervals):
    # type: (List[List[float]], np.ndarray, float, List[Tuple[float, float]]) -> List[Optional[List[float]]]
    """弧長区間 intervals に入る点を None にする (内側境界線の接続口)."""
    def inside(value):
        for a, b in intervals:
            if a <= value < b or a <= value + total < b:
                return True
        return False
    return [None if inside(s[i]) else p for i, p in enumerate(line)]


def write_lines_js(geom, path=OUT_LINES):
    # type: (dict, str) -> None
    """
    js/ideal_lane_detector.js は COURSE_LINES の null を「そこに線が無い」
    として扱い, 理想検出のリアリティ (破線・合流部では線が見えない) を作って
    いる. 連続線を吐くとその性質が失われるので, 描画と同じ破線パターンと
    接続口を null として入れる.
    """
    lines = geom["_lines"]
    path_pts = np.array(geom["centerPath"])
    seg = np.hypot(*(np.roll(path_pts, -1, axis=0) - path_pts).T)
    s = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
    total = geom["lengthM"]
    out = {
        "outer": lines["outer"],
        "center": dashed_line(lines["center"], s, DASH_MARK_M, DASH_GAP_M),
        "inner": gapped_line(lines["inner"], s, total, geom["innerGaps"]),
    }
    with open(path, "w") as f:
        f.write("// Generated by web_simulator/tools/build_course.py -- do not edit.\n")
        f.write("// Outer-loop white lines in world (ROS) coordinates, derived from\n")
        f.write("// COURSE_GEOMETRY.centerPath by offsetting +/-%.2f m -- not traced\n" % LANE_WIDTH_M)
        f.write("// from the texture, so these coincide exactly with what is drawn.\n")
        f.write("// null = no line there (the centre line's dash gaps, and the inner\n")
        f.write("// boundary's junction openings), same convention as before.\n")
        f.write("export const COURSE_LINES = ")
        json.dump(out, f, separators=(",", ":"))
        f.write(";\n")


def erase_lines(image, radii, center, n_rays, corridor_px, threshold, fill_bgr):
    # type: (np.ndarray, dict, Tuple[int, int], int, float, int, Tuple[int, int, int]) -> np.ndarray
    """
    抽出された各線の位置を中心に corridor_px の回廊をとり, その中の
    「輝度 threshold 超」の画素だけを fill_bgr で塗り潰す.
    回廊外と非白画素には触れないので, 回廊に入り込んだ芝生やアスファルトは残る.

    隣り合うレイ間の接線方向の隙間を閉じるために, 各線について隣接する
    レイペア間で thickness = 1.5*corridor_px の cv2.line を描画する. この
    thickness は線の中心から両側に効くため, 実際の法線方向の消去半幅は
    0.75*corridor_px であり, corridor_px そのものではない.
    """
    out = image.copy()
    h, w = out.shape[:2]
    gray = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
    mask = np.zeros((h, w), np.uint8)
    cx, cy = center
    # Thickness for cv2.line to connect adjacent rays
    # This must be large enough to close the tangential gaps between rays (~2.4 px at outer radius)
    # but small enough to not exceed corridor_px radially. Use 1.5*corridor_px.
    thickness_px = max(1, int(round(1.5 * corridor_px)))

    # First, mark each traced ray position to ensure all traced points are erased
    for name in ("outer", "center", "inner"):
        for k, r in enumerate(radii[name]):
            if r is None:
                continue
            theta = 2 * np.pi * k / n_rays
            dx, dy = np.cos(theta), np.sin(theta)
            x = int(round(cx + r * dx))
            y = int(round(cy + r * dy))
            if 0 <= x < w and 0 <= y < h:
                mask[y, x] = 1

    # Then, draw lines between adjacent traced positions to close tangential gaps
    for name in ("outer", "center", "inner"):
        for k in range(n_rays):
            r_k = radii[name][k]
            r_next = radii[name][(k + 1) % n_rays]
            # Both radii must be non-None to draw a connecting segment
            if r_k is None or r_next is None:
                continue
            # Current ray endpoint
            theta_k = 2 * np.pi * k / n_rays
            dx_k, dy_k = np.cos(theta_k), np.sin(theta_k)
            pt_k = (int(round(cx + r_k * dx_k)), int(round(cy + r_k * dy_k)))
            # Next ray endpoint
            theta_next = 2 * np.pi * (k + 1) / n_rays
            dx_next, dy_next = np.cos(theta_next), np.sin(theta_next)
            pt_next = (int(round(cx + r_next * dx_next)), int(round(cy + r_next * dy_next)))
            # Draw line segment with thickness to close the gap
            cv2.line(mask, pt_k, pt_next, 1, thickness=thickness_px)

    out[(mask == 1) & (gray > threshold)] = fill_bgr
    return out


def course_pose(geom):
    # type: (dict) -> Tuple[float, float]
    """
    背景テクスチャ (png) を置く板の位置 (js/simulator.js の COURSE_POSE に
    書き写す値) を返す.

    radii_to_world() はピクセル -> 世界座標の変換を SRC_POSE (無変換) と
    COURSE_WIDTH_M (= SRC_WIDTH_M * SCALE_K, 実寸化のスケールはここに
    集約されている) だけで行っている. つまり SCALE_K は COURSE_WIDTH_M に
    既に入っており, SRC_POSE 自体はスケールしない. build_geometry() が
    その後で基準パスを START_WORLD にアンカリングするために平行移動した分
    (geom["_shift"]) だけ, 板の位置も同じだけずらせばよい -- SCALE_K を
    もう一度掛けてはいけない (掛けると背景テクスチャの位置だけが実寸化の
    スケールぶんズレて, 生成した線がテクスチャの線からはみ出る).
    """
    shift = geom["_shift"]
    return SRC_POSE[0] + shift[0], SRC_POSE[1] + shift[1]


def main():
    geom = build_geometry()
    path = np.array(geom["centerPath"])
    kappa = np.abs(curvature_closed(path))
    traced_center = np.array([p for p in geom["_traced"]["center"] if p])
    dev = nearest_distance(traced_center, path)
    print("centre path : %d pts, %.2f m, min radius %.1f m" % (len(path), geom["lengthM"], 1.0 / kappa.max()))
    print("smoothing   : RMS dev from traced %.3f m, max %.3f m" % (float(np.sqrt((dev ** 2).mean())), float(dev.max())))
    for side in ("outer", "inner"):
        d = nearest_distance(np.array(geom["_lines"][side]), path)
        print("lane width  : %-6s %.4f m (min %.4f / max %.4f)" % (side, d.mean(), d.min(), d.max()))
    print("inner gaps  : %s" % geom["innerGaps"])
    print("start pose  : %s" % geom["startPose"])
    pose_x, pose_y = course_pose(geom)
    print("COURSE_POSE : x: %.4f, y: %.4f   <- copy into js/simulator.js" % (pose_x, pose_y))
    write_geometry_js(geom)
    write_lines_js(geom)
    print("wrote", OUT_GEOM)
    print("wrote", OUT_LINES)
    image = cv2.imread(SRC_IMG)
    corridor_px = ERASE_CORRIDOR_M / (COURSE_WIDTH_M / image.shape[1])
    base = erase_lines(image, geom["_radii"], RAY_CENTER, N_RAYS, corridor_px,
                       WHITE_THRESHOLD, ASPHALT_BGR)
    cv2.imwrite(OUT_IMG, base)
    print("wrote", OUT_IMG, "(erase corridor %.2f m = %.1f px)" % (ERASE_CORRIDOR_M, corridor_px))


if __name__ == "__main__":
    main()
