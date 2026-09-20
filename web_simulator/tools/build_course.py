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
from typing import List, Optional, Sequence, Tuple

import numpy as np

# --- 寸法 (仕様書 1/2/3 節) ---
LANE_WIDTH_M = 3.5          # 中央線 <-> 境界線
LINE_WIDTH_M = 0.15         # 白線の幅
HARMONICS = 30              # FFT ローパスで残す次数
RESAMPLE_STEP_M = 0.10      # 基準パスの等弧長間隔


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
