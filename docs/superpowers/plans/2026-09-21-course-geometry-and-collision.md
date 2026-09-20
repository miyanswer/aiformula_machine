# web_simulator コース幾何データ化・実寸化と当たり判定 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** web_simulator の外周コースを「白線幅 15 cm・車線幅 3.5 m」の幾何定義から生成し、障害物衝突（物理的に阻止）とコース逸脱（警告のみ）の当たり判定を追加する。

**Architecture:** ビルド時の Python ツールが PNG から抽出した中央線を FFT 平滑化して基準パスにし、そこから ±3.5 m のオフセット線と `course_geometry.js` / `course_lines.js` を生成する。同じツールが元 PNG から外周 3 本線だけを消した背景画像を出力する。ランタイムは基準パスから幅 15 cm のリボンメッシュとして白線を描き、当たり判定も同じ基準パス（逸脱）と明示的な衝突円リスト（障害物）から計算する。

**Tech Stack:** Python 3.9 (stdlib unittest + numpy 1.26 + opencv 4.10), three.js r160 (ES modules, バンドラなし), ブラウザ上での検証スクリプト

**Spec:** `docs/superpowers/specs/2026-09-21-course-geometry-and-collision-design.md`

## Global Constraints

- **Python は 3.9.6**（ホストの `/usr/bin/python3`）。`match` 文、`X | Y` 形式の型注釈、`list[int]` 形式の組み込みジェネリクス注釈は使わない。`typing.Optional` / `typing.List` を使う
- **pytest はホストに存在しない。** テストは stdlib の `unittest` で書き、`python3 -m unittest` で走らせる
- **node もホストに存在しない。** JS のテストは実行できないため、JS の検証はブラウザ上で `tools/verify_course.js` を動的 import して行う
- **vendor/ は three.js r160 のオフライン同梱コピー**。`vendor/three/examples/jsm/` には `ColladaLoader.js` と `TGALoader.js` しかない。新しい addon を前提にしない
- 数値定数はすべて仕様書に記載の実測値を使う: `LANE_WIDTH_M = 3.5`, `LINE_WIDTH_M = 0.15`, `COURSE_WIDTH_M = 106.80`, `HARMONICS = 30`, `RESAMPLE_STEP_M = 0.10`, `ERASE_CORRIDOR_M = 0.45`, `WHITE_THRESHOLD = 195`, `ASPHALT_BGR = (156, 150, 156)`, `LINE_BGR = (219, 212, 221)`, `DASH_MARK_M = 2.8`, `DASH_GAP_M = 2.6`, `INNER_GAP_MIN_M = 1.5`, `VEHICLE_HALF_WIDTH = 0.40`
- コメントは既存 `web_simulator/js/*.js` に合わせて**英語**で書く（README と docs は日本語）
- コミットメッセージの末尾に `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>` を付ける

---

## File Structure

| ファイル | 責務 |
|---|---|
| `web_simulator/tools/build_course.py`（新規） | PNG 抽出 → 平滑化 → 実寸化 → `course_geometry.js` / `course_lines.js` / `shihou_cource_base.png` 生成。純粋関数群 + `main()` |
| `web_simulator/tools/test_build_course.py`（新規） | 上記の純粋関数と、実 PNG に対する結合テスト |
| `web_simulator/tools/extract_course_lines.py`（削除） | `build_course.py` に置き換え |
| `web_simulator/js/course_geometry.js`（生成物） | 基準パスと寸法定数。手で編集しない |
| `web_simulator/js/course_lines.js`（生成物・既存を置換） | 3 本線の点列。理想検出モードの入力 |
| `web_simulator/png/shihou_cource_base.png`（生成物） | 外周 3 本線を消した背景テクスチャ |
| `web_simulator/js/course.js`（変更） | 縮尺定数、背景テクスチャ読み込み、白線リボンメッシュ生成 |
| `web_simulator/js/collision.js`（新規） | 純粋な 2D 当たり判定: 円衝突解決、基準パスへの最近傍・横方向距離、逸脱カウンタ |
| `web_simulator/js/course_props.js`（変更） | `MYLAPS_POSE` 再計算、衝突円リスト、デバッグ用の露出 |
| `web_simulator/js/simulator.js`（変更） | 縮尺関連定数の更新、衝突と逸脱の配線、HUD 更新 |
| `web_simulator/index.html`（変更） | 逸脱警告・累計回数・接触表示の HUD 要素 |
| `web_simulator/tools/verify_course.js`（新規） | ブラウザ上で走らせる検証チェック集 |
| `web_simulator/README.md`（変更） | 実寸・生成ツール・当たり判定の節 |

---

### Task 1: ビルドツールの幾何コア（純粋関数）

**Files:**
- Create: `web_simulator/tools/build_course.py`
- Test: `web_simulator/tools/test_build_course.py`

**Interfaces:**
- Consumes: なし
- Produces:
  - `fill_closed(line: List[Optional[Sequence[float]]], n_rays: int) -> np.ndarray` — (N,2)
  - `lowpass_closed(curve: np.ndarray, harmonics: int) -> np.ndarray` — (N,2)
  - `resample_closed(curve: np.ndarray, step_m: float) -> Tuple[np.ndarray, float]` — ((M,2), 全長)
  - `normals_closed(path: np.ndarray) -> np.ndarray` — (M,2) 単位左手法線
  - `offset_closed(path: np.ndarray, distance: float) -> np.ndarray` — (M,2)
  - `curvature_closed(path: np.ndarray) -> np.ndarray` — (M,)
  - `nearest_distance(points: np.ndarray, polyline: np.ndarray) -> np.ndarray` — (K,)

- [ ] **Step 1: 失敗するテストを書く**

`web_simulator/tools/test_build_course.py`:

```python
"""build_course.py の幾何コアのユニットテスト (python3 -m unittest で実行)."""

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from build_course import (  # noqa: E402
    curvature_closed, fill_closed, lowpass_closed, nearest_distance,
    normals_closed, offset_closed, resample_closed,
)


def unit_circle(n, radius=10.0):
    th = np.arange(n) / n * 2 * np.pi
    return np.stack([radius * np.cos(th), radius * np.sin(th)], axis=1)


class TestFillClosed(unittest.TestCase):
    def test_fills_none_by_periodic_interpolation(self):
        circle = unit_circle(8)
        line = [list(p) for p in circle]
        line[3] = None
        out = fill_closed(line, 8)
        self.assertEqual(out.shape, (8, 2))
        expected = (circle[2] + circle[4]) / 2
        np.testing.assert_allclose(out[3], expected, atol=1e-9)

    def test_fills_gap_that_wraps_around_index_zero(self):
        circle = unit_circle(8)
        line = [list(p) for p in circle]
        line[0] = None
        out = fill_closed(line, 8)
        expected = (circle[7] + circle[1]) / 2
        np.testing.assert_allclose(out[0], expected, atol=1e-9)


class TestLowpassClosed(unittest.TestCase):
    def test_removes_high_frequency_jitter(self):
        n = 720
        clean = unit_circle(n)
        rng = np.random.default_rng(0)
        noisy = clean + rng.normal(0, 0.3, size=clean.shape)
        smoothed = lowpass_closed(noisy, 30)
        raw_err = np.hypot(*(noisy - clean).T).mean()
        smooth_err = np.hypot(*(smoothed - clean).T).mean()
        self.assertLess(smooth_err, raw_err / 3.0)

    def test_leaves_a_pure_circle_unchanged(self):
        clean = unit_circle(720)
        np.testing.assert_allclose(lowpass_closed(clean, 30), clean, atol=1e-6)


class TestResampleClosed(unittest.TestCase):
    def test_produces_equal_arc_length_spacing(self):
        out, total = resample_closed(unit_circle(720), 0.10)
        seg = np.hypot(*(np.roll(out, -1, axis=0) - out).T)
        self.assertLess(seg.max() - seg.min(), 1e-6)
        self.assertAlmostEqual(total, 2 * math.pi * 10.0, delta=0.01)

    def test_total_length_is_the_polyline_length(self):
        square = np.array([[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]])
        _, total = resample_closed(square, 0.10)
        self.assertAlmostEqual(total, 16.0, delta=1e-9)


class TestNormalsAndOffset(unittest.TestCase):
    def test_normals_are_unit_length(self):
        n = normals_closed(unit_circle(360))
        np.testing.assert_allclose(np.hypot(*n.T), 1.0, atol=1e-9)

    def test_offsetting_a_ccw_circle_shrinks_it_on_the_left(self):
        # CCW の円では左手法線は内向き。+1.0 のオフセットで半径は 9.0 になる。
        out = offset_closed(unit_circle(3600), 1.0)
        np.testing.assert_allclose(np.hypot(*out.T), 9.0, atol=1e-3)

    def test_offsetting_the_other_way_grows_it(self):
        out = offset_closed(unit_circle(3600), -1.0)
        np.testing.assert_allclose(np.hypot(*out.T), 11.0, atol=1e-3)


class TestCurvature(unittest.TestCase):
    def test_circle_curvature_is_one_over_radius(self):
        k = curvature_closed(unit_circle(3600, radius=10.0))
        np.testing.assert_allclose(np.abs(k), 0.1, rtol=1e-3)


class TestNearestDistance(unittest.TestCase):
    def test_distance_from_offset_circle_to_original(self):
        path = unit_circle(3600, radius=10.0)
        probe = unit_circle(720, radius=13.5)
        d = nearest_distance(probe, path)
        np.testing.assert_allclose(d, 3.5, atol=1e-3)

    def test_point_on_the_polyline_has_zero_distance(self):
        path = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])
        d = nearest_distance(np.array([[5.0, 0.0]]), path)
        self.assertAlmostEqual(float(d[0]), 0.0, delta=1e-9)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `cd web_simulator/tools && python3 -m unittest test_build_course -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'build_course'`

- [ ] **Step 3: 幾何コアを実装**

`web_simulator/tools/build_course.py`:

```python
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
    dx = np.gradient(path[:, 0])
    dy = np.gradient(path[:, 1])
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
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
```

- [ ] **Step 4: テストが通ることを確認**

Run: `cd web_simulator/tools && python3 -m unittest test_build_course -v`
Expected: PASS（12 tests）

- [ ] **Step 5: コミット**

```bash
git add web_simulator/tools/build_course.py web_simulator/tools/test_build_course.py
git commit -m "$(cat <<'EOF'
feat(web_simulator): add closed-curve geometry core for course generation

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 2: 抽出・実寸化と `course_geometry.js` / `course_lines.js` の生成

**Files:**
- Modify: `web_simulator/tools/build_course.py`（Task 1 の続き）
- Modify: `web_simulator/tools/test_build_course.py`
- Create（生成物）: `web_simulator/js/course_geometry.js`, `web_simulator/js/course_lines.js`
- Delete: `web_simulator/tools/extract_course_lines.py`

**Interfaces:**
- Consumes: Task 1 の `fill_closed` / `lowpass_closed` / `resample_closed` / `offset_closed` / `curvature_closed` / `nearest_distance`
- Produces:
  - `extract_ray_radii(white: np.ndarray, center: Tuple[int, int], n_rays: int) -> dict` — キー `"outer"` / `"center"` / `"inner"`、値は長さ `n_rays` の `List[Optional[float]]`（レイ上の半径 px）
  - `radii_to_world(radii, center, n_rays, image_shape, width_m, pose) -> List[Optional[List[float]]]`
  - `gap_intervals(line: List[Optional[object]], seg_lengths: np.ndarray, min_gap_m: float) -> List[Tuple[float, float]]`
  - `build_geometry(gray_white, ...) -> dict` — `course_geometry.js` に書く内容そのもの
  - `js/course_geometry.js` が `export const COURSE_GEOMETRY = {...}` を出す
  - `dashed_line(line, s, total, mark_m, gap_m) -> List[Optional[List[float]]]`
  - `gapped_line(line, s, total, intervals) -> List[Optional[List[float]]]`
  - `js/course_lines.js` が従来どおり `export const COURSE_LINES = {"outer":[...], "center":[...], "inner":[...]}` を出す。**中央線の破線の切れ目と内側境界線の接続口は `null`**（`ideal_lane_detector.js` が「そこに線が無い」判定に使っているため）

- [ ] **Step 1: 失敗するテストを追加**

`web_simulator/tools/test_build_course.py` の末尾（`if __name__` の直前）に追加:

```python
class TestGapIntervals(unittest.TestCase):
    def test_reports_only_gaps_above_the_threshold(self):
        line = [1, 1, None, 1, 1, None, None, None, None, 1]
        seg = np.full(10, 1.0)  # 1 点 = 1 m
        got = gap_intervals(line, seg, min_gap_m=1.5)
        self.assertEqual(len(got), 1)
        self.assertAlmostEqual(got[0][0], 5.0, delta=1e-9)
        self.assertAlmostEqual(got[0][1], 9.0, delta=1e-9)

    def test_gap_wrapping_past_the_end_is_reported_once(self):
        line = [None, None, 1, 1, 1, 1, 1, 1, None, None]
        seg = np.full(10, 1.0)
        got = gap_intervals(line, seg, min_gap_m=1.5)
        self.assertEqual(len(got), 1)
        self.assertAlmostEqual(got[0][0], 8.0, delta=1e-9)
        self.assertAlmostEqual(got[0][1], 12.0, delta=1e-9)  # 全長 10 m を跨ぐ


class TestDashAndGapNulls(unittest.TestCase):
    def test_dashed_line_blanks_the_gap_part_of_each_pitch(self):
        line = [[float(i), 0.0] for i in range(10)]
        s = np.arange(10, dtype=float)          # 1 点 = 1 m
        out = dashed_line(line, s, 10.0, mark_m=3.0, gap_m=2.0)   # pitch 5 m
        drawn = [i for i, p in enumerate(out) if p is not None]
        self.assertEqual(drawn, [0, 1, 2, 5, 6, 7])

    def test_gapped_line_blanks_the_listed_intervals(self):
        line = [[float(i), 0.0] for i in range(10)]
        s = np.arange(10, dtype=float)
        out = gapped_line(line, s, 10.0, [(3.0, 6.0)])
        drawn = [i for i, p in enumerate(out) if p is not None]
        self.assertEqual(drawn, [0, 1, 2, 6, 7, 8, 9])


class TestGeneratedCourseLinesKeepNulls(unittest.TestCase):
    """ideal_lane_detector.js が null を「線が無い」判定に使っているので落とさないこと."""

    @classmethod
    def setUpClass(cls):
        cls.geom = build_geometry()

    def test_centre_line_is_dashed(self):
        path_pts = np.array(self.geom["centerPath"])
        seg = np.hypot(*(np.roll(path_pts, -1, axis=0) - path_pts).T)
        s = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
        out = dashed_line(self.geom["_lines"]["center"], s, self.geom["lengthM"],
                          DASH_MARK_M, DASH_GAP_M)
        drawn = sum(1 for p in out if p is not None)
        ratio = drawn / float(len(out))
        expected = DASH_MARK_M / (DASH_MARK_M + DASH_GAP_M)
        self.assertAlmostEqual(ratio, expected, delta=0.02)

    def test_outer_line_has_no_nulls(self):
        self.assertTrue(all(p is not None for p in self.geom["_lines"]["outer"]))


class TestBuildGeometryOnTheRealCourse(unittest.TestCase):
    """実 PNG を通した結合テスト. 数秒かかる."""

    @classmethod
    def setUpClass(cls):
        cls.geom = build_geometry()

    def test_lane_width_is_exactly_3_5_metres(self):
        path = np.array(self.geom["centerPath"])
        for side in ("outer", "inner"):
            line = np.array(self.geom["_lines"][side])
            d = nearest_distance(line, path)
            self.assertAlmostEqual(float(d.mean()), 3.5, delta=0.001)
            self.assertLess(abs(float(d.max()) - 3.5), 0.001)
            self.assertLess(abs(float(d.min()) - 3.5), 0.001)

    def test_offset_lines_do_not_self_intersect(self):
        path = np.array(self.geom["centerPath"])
        min_radius = 1.0 / np.abs(curvature_closed(path)).max()
        self.assertGreater(min_radius, 3.5)

    def test_total_length_matches_the_spec(self):
        self.assertAlmostEqual(self.geom["lengthM"], 246.9, delta=2.0)

    def test_start_pose_sits_on_the_centre_line(self):
        path = np.array(self.geom["centerPath"])
        start = np.array([[self.geom["startPose"]["x"], self.geom["startPose"]["y"]]])
        self.assertLess(float(nearest_distance(start, path)[0]), 0.05)

    def test_start_pose_is_the_world_origin_anchor(self):
        self.assertAlmostEqual(self.geom["startPose"]["x"], 0.0, delta=0.05)
        self.assertAlmostEqual(self.geom["startPose"]["y"], -1.6, delta=0.05)

    def test_inner_gaps_are_preserved(self):
        self.assertGreaterEqual(len(self.geom["innerGaps"]), 3)
        for s0, s1 in self.geom["innerGaps"]:
            self.assertGreater(s1 - s0, 1.5)
```

`import` 行に `build_geometry` と `gap_intervals` を足す:

```python
from build_course import (  # noqa: E402
    DASH_GAP_M, DASH_MARK_M, build_geometry, curvature_closed, dashed_line, fill_closed,
    gap_intervals, gapped_line, lowpass_closed, nearest_distance, normals_closed,
    offset_closed, resample_closed,
)
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `cd web_simulator/tools && python3 -m unittest test_build_course -v`
Expected: FAIL — `ImportError: cannot import name 'build_geometry'`

- [ ] **Step 3: 抽出と実寸化を実装**

`build_course.py` の定数ブロックに追記:

```python
import json
import os

import cv2

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
```

純粋関数を追加:

```python
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
```

続いて組み立て:

```python
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

    # 4. 内側境界線の接続口 (抽出データ上の長い欠損) を弧長区間で持ち越す
    inner_rolled = traced["inner"][anchor_idx * 0:]  # 抽出は N_RAYS 刻み, 弧長は概算で十分
    ray_seg = np.hypot(*(np.roll(filled, -1, axis=0) - filled).T) * (total / float(np.hypot(
        *(np.roll(filled, -1, axis=0) - filled).T).sum()))
    inner_gaps = gap_intervals(inner_rolled, ray_seg, INNER_GAP_MIN_M)

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
        "innerGaps": [[round(a, 3), round(b, 3)] for a, b in inner_gaps],
        "centerPath": [[round(float(p[0]), 3), round(float(p[1]), 3)] for p in path],
        "_lines": {k: [[round(float(p[0]), 3), round(float(p[1]), 3)] for p in v]
                   for k, v in lines.items()},
        "_traced": traced,
        "_radii": radii,
        "_shift": [float(shift[0]), float(shift[1])],
    }
```

`_` 始まりのキーは JS に書き出さない中間データ（テストと後続タスクが使う）。

- [ ] **Step 4: テストが通ることを確認**

Run: `cd web_simulator/tools && python3 -m unittest test_build_course -v`
Expected: PASS（24 tests）。結合テストは 5〜15 秒かかる

- [ ] **Step 5: JS 生成を実装**

`build_course.py` の末尾に追加:

```python
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


def dashed_line(line, s, total, mark_m, gap_m):
    # type: (List[List[float]], np.ndarray, float, float, float) -> List[Optional[List[float]]]
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
        "center": dashed_line(lines["center"], s, total, DASH_MARK_M, DASH_GAP_M),
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
    # simulator.js にそのまま書き写す値. 仕様書の計算値 (14.119, 37.927) に
    # 基準パスのアンカリング平行移動を足したもの.
    pose_x = 13.22 * SCALE_K + geom["_shift"][0]
    pose_y = -1.6 + 37.01 * SCALE_K + geom["_shift"][1]
    print("COURSE_POSE : x: %.4f, y: %.4f   <- copy into js/simulator.js" % (pose_x, pose_y))
    write_geometry_js(geom)
    write_lines_js(geom)
    print("wrote", OUT_GEOM)
    print("wrote", OUT_LINES)


if __name__ == "__main__":
    main()
```

- [ ] **Step 6: ツールを実行して診断値を確認**

Run: `python3 web_simulator/tools/build_course.py`
Expected: `lane width` が outer / inner とも `3.5000 m (min 3.4995 / max 3.5005)` 相当、`min radius` が 10 m 前後、`smoothing` の RMS 偏差が 0.05 m 前後（下表の平均偏差 0.034 m に対する RMS。裾が長いので RMS の方が大きい）。`js/course_geometry.js` と `js/course_lines.js` が書き換わる

- [ ] **Step 7: 旧ツールを削除**

```bash
git rm web_simulator/tools/extract_course_lines.py
```

- [ ] **Step 8: コミット**

```bash
git add web_simulator/tools/build_course.py web_simulator/tools/test_build_course.py \
        web_simulator/js/course_geometry.js web_simulator/js/course_lines.js
git commit -m "$(cat <<'EOF'
feat(web_simulator): generate course geometry with exact 3.5m lanes

course_lines.js is now derived from the smoothed reference path instead of
traced from the texture, so the drawn course and the course the ideal
detector sees are the same geometry by construction.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 3: 背景テクスチャ（外周 3 本線の消去）

**Files:**
- Modify: `web_simulator/tools/build_course.py`
- Modify: `web_simulator/tools/test_build_course.py`
- Create（生成物）: `web_simulator/png/shihou_cource_base.png`

**Interfaces:**
- Consumes: Task 2 の `build_geometry()`（`_radii` キー）
- Produces: `erase_lines(image: np.ndarray, radii: dict, center, n_rays, corridor_px: float, threshold: int, fill_bgr) -> np.ndarray`

- [ ] **Step 1: 失敗するテストを追加**

`test_build_course.py` に追加（`import` に `erase_lines`, `ERASE_CORRIDOR_M`, `ASPHALT_BGR`, `RAY_CENTER`, `N_RAYS`, `COURSE_WIDTH_M`, `WHITE_THRESHOLD` を足す）:

```python
class TestEraseLines(unittest.TestCase):
    def test_replaces_white_pixels_inside_the_corridor(self):
        image = np.full((200, 200, 3), (156, 150, 156), dtype=np.uint8)
        image[100, 150] = (255, 255, 255)          # レイ 0 (右向き) 上, 半径 50
        radii = {"outer": [50.0] + [None] * 7, "center": [None] * 8, "inner": [None] * 8}
        out = erase_lines(image, radii, (100, 100), 8, 4.0, 195, (156, 150, 156))
        np.testing.assert_array_equal(out[100, 150], (156, 150, 156))

    def test_leaves_non_white_pixels_alone(self):
        image = np.full((200, 200, 3), (156, 150, 156), dtype=np.uint8)
        image[100, 150] = (127, 185, 165)          # 芝生色
        radii = {"outer": [50.0] + [None] * 7, "center": [None] * 8, "inner": [None] * 8}
        out = erase_lines(image, radii, (100, 100), 8, 4.0, 195, (156, 150, 156))
        np.testing.assert_array_equal(out[100, 150], (127, 185, 165))

    def test_leaves_white_pixels_outside_the_corridor_alone(self):
        image = np.full((200, 200, 3), (156, 150, 156), dtype=np.uint8)
        image[100, 180] = (255, 255, 255)          # 半径 80, 回廊 (50±4) の外
        radii = {"outer": [50.0] + [None] * 7, "center": [None] * 8, "inner": [None] * 8}
        out = erase_lines(image, radii, (100, 100), 8, 4.0, 195, (156, 150, 156))
        np.testing.assert_array_equal(out[100, 180], (255, 255, 255))


class TestErasedCourseImage(unittest.TestCase):
    """実 PNG に対する結合テスト."""

    def test_traced_line_positions_are_no_longer_white(self):
        image = cv2.imread(SRC_IMG)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        radii = extract_ray_radii(gray > WHITE_THRESHOLD, RAY_CENTER, N_RAYS)
        corridor_px = ERASE_CORRIDOR_M / (COURSE_WIDTH_M / image.shape[1])
        out = erase_lines(image, radii, RAY_CENTER, N_RAYS, corridor_px,
                          WHITE_THRESHOLD, ASPHALT_BGR)
        out_gray = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
        cx, cy = RAY_CENTER
        remaining = 0
        for name in ("outer", "center", "inner"):
            for k, r in enumerate(radii[name]):
                if r is None:
                    continue
                th = 2 * np.pi * k / N_RAYS
                px = int(round(cx + r * np.cos(th)))
                py = int(round(cy + r * np.sin(th)))
                if out_gray[py, px] > WHITE_THRESHOLD:
                    remaining += 1
        self.assertEqual(remaining, 0)

    def test_the_kerb_line_outside_the_loop_survives(self):
        """外側の二重線 (縁石線) は残すこと. 消えた割合が 5% 未満であること."""
        image = cv2.imread(SRC_IMG)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        white = gray > WHITE_THRESHOLD
        before = int(white.sum())
        radii = extract_ray_radii(white, RAY_CENTER, N_RAYS)
        corridor_px = ERASE_CORRIDOR_M / (COURSE_WIDTH_M / image.shape[1])
        out = erase_lines(image, radii, RAY_CENTER, N_RAYS, corridor_px,
                          WHITE_THRESHOLD, ASPHALT_BGR)
        after = int((cv2.cvtColor(out, cv2.COLOR_BGR2GRAY) > WHITE_THRESHOLD).sum())
        # 外周 3 本線だけが消えるので, 画像全体の白画素が半分以上残るはず
        self.assertGreater(after, before * 0.5)
```

- [ ] **Step 2: テストが失敗することを確認**

Run: `cd web_simulator/tools && python3 -m unittest test_build_course -v`
Expected: FAIL — `ImportError: cannot import name 'erase_lines'`

- [ ] **Step 3: 実装**

`build_course.py` の定数に追加:

```python
ERASE_CORRIDOR_M = 0.45           # 抽出線からの法線方向の消去範囲
ASPHALT_BGR = (156, 150, 156)     # 元 PNG のアスファルト色
```

関数を追加:

```python
def erase_lines(image, radii, center, n_rays, corridor_px, threshold, fill_bgr):
    # type: (np.ndarray, dict, Tuple[int, int], int, float, int, Tuple[int, int, int]) -> np.ndarray
    """
    抽出された各線の位置を中心に corridor_px の回廊をとり, その中の
    「輝度 threshold 超」の画素だけを fill_bgr で塗り潰す.
    回廊外と非白画素には触れないので, 回廊に入り込んだ芝生やアスファルトは残る.
    """
    out = image.copy()
    h, w = out.shape[:2]
    gray = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
    mask = np.zeros((h, w), np.uint8)
    cx, cy = center
    span = np.arange(-corridor_px, corridor_px + 0.5, 0.5)
    for name in ("outer", "center", "inner"):
        for k, r in enumerate(radii[name]):
            if r is None:
                continue
            theta = 2 * np.pi * k / n_rays
            dx, dy = np.cos(theta), np.sin(theta)
            xs = (cx + (r + span) * dx).round().astype(int)
            ys = (cy + (r + span) * dy).round().astype(int)
            ok = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
            mask[ys[ok], xs[ok]] = 1
    out[(mask == 1) & (gray > threshold)] = fill_bgr
    return out
```

レイ間の隙間を埋めるため、回廊マスクを少し膨張させる（1440 本のレイでも外周では隣り合うレイの間隔が約 2.4 px 開く）:

```python
    mask = cv2.dilate(mask, np.ones((3, 3), np.uint8), iterations=2)
    out[(mask == 1) & (gray > threshold)] = fill_bgr
    return out
```

（`mask` の膨張は `for` ループの後、`out[...]` の前に入れる。上の 2 行で元の代入行を置き換える）

`main()` の `write_lines_js(geom)` の直後に追加:

```python
    image = cv2.imread(SRC_IMG)
    corridor_px = ERASE_CORRIDOR_M / (COURSE_WIDTH_M / image.shape[1])
    base = erase_lines(image, geom["_radii"], RAY_CENTER, N_RAYS, corridor_px,
                       WHITE_THRESHOLD, ASPHALT_BGR)
    cv2.imwrite(OUT_IMG, base)
    print("wrote", OUT_IMG, "(erase corridor %.2f m = %.1f px)" % (ERASE_CORRIDOR_M, corridor_px))
```

- [ ] **Step 4: テストが通ることを確認**

Run: `cd web_simulator/tools && python3 -m unittest test_build_course -v`
Expected: PASS（29 tests）

- [ ] **Step 5: ツールを実行して画像を生成**

Run: `python3 web_simulator/tools/build_course.py`
Expected: `wrote .../png/shihou_cource_base.png (erase corridor 0.45 m = 4.3 px)`

- [ ] **Step 6: 目視確認**

Run: `python3 -c "import cv2; im=cv2.imread('web_simulator/png/shihou_cource_base.png'); cv2.imwrite('/tmp/base_check.png', cv2.resize(im,(768,614)))"`
`/tmp/base_check.png` を開き、外周ループの 3 本線が消えていて、内側の交差点・横断歩道・駐車枠・芝生島と外側の縁石線が残っていることを確認する

- [ ] **Step 7: コミット**

```bash
git add web_simulator/tools/build_course.py web_simulator/tools/test_build_course.py \
        web_simulator/png/shihou_cource_base.png
git commit -m "$(cat <<'EOF'
feat(web_simulator): generate a base texture with the outer-loop lines erased

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 4: コースの実寸化と白線リボンメッシュ

**Files:**
- Modify: `web_simulator/js/course.js`

**Interfaces:**
- Consumes: `COURSE_GEOMETRY`（Task 2）
- Produces:
  - `COURSE_WIDTH_M: number`（106.80）、`COURSE_DEPTH_M: number`
  - `createCourseTexture(): THREE.Texture`（背景を `shihou_cource_base.png` に変更）
  - `createCourseLines(): THREE.Group` — 3 本の白線リボンを含む Group。`rosRoot` に直接 add できる ROS 座標

- [ ] **Step 1: 実装**

`web_simulator/js/course.js` を全面的に書き換える:

```js
import * as THREE from 'three';
import { COURSE_GEOMETRY } from './course_geometry.js';

// Ground texture: the user's own course layout image with the outer loop's
// three white lines erased (web_simulator/tools/build_course.py). Those three
// lines are drawn as geometry instead -- see createCourseLines() -- so that
// the 15cm line width and 3.5m lane width are exact rather than limited by
// the texture's 10.4cm/px resolution. Everything else in the image (the
// inner roads, crossings, parking bays and grass islands) is still the
// texture, scaled so its outer loop lands on the generated geometry.
const COURSE_IMAGE_URL = 'png/shihou_cource_base.png'; // relative to index.html
const COURSE_IMAGE_ASPECT = 1024 / 819; // width / height, from the source PNG

// 100m -> 106.80m: the traced lane width averaged 3.2772m, so scaling by
// 3.5 / 3.2772 = 1.0680 puts the texture's own loop on top of the generated
// 3.5m lanes. See tools/build_course.py SCALE_K.
export const COURSE_WIDTH_M = 106.80;
export const COURSE_DEPTH_M = COURSE_WIDTH_M / COURSE_IMAGE_ASPECT;

const LINE_COLOR = 0xdbd4dd; // the source PNG's own white-line colour

const textureLoader = new THREE.TextureLoader();

export function createCourseTexture() {
  const texture = textureLoader.load(COURSE_IMAGE_URL, undefined, undefined, (err) =>
    console.error(`Failed to load course image ${COURSE_IMAGE_URL}`, err)
  );
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = 4;
  return texture;
}

// --- White lines, generated from COURSE_GEOMETRY.centerPath ---------------

// Unit left-hand normal at each point of a closed polyline (central
// difference, so it matches the tangent the build tool used).
function normalsClosed(path) {
  const n = path.length;
  return path.map((_, i) => {
    const a = path[(i - 1 + n) % n];
    const b = path[(i + 1) % n];
    const tx = b[0] - a[0];
    const ty = b[1] - a[1];
    const len = Math.hypot(tx, ty) || 1e-12;
    return [-ty / len, tx / len];
  });
}

function offsetClosed(path, distance) {
  const normals = normalsClosed(path);
  return path.map((p, i) => [p[0] + normals[i][0] * distance, p[1] + normals[i][1] * distance]);
}

// Cumulative arc length of a closed polyline, plus its total.
function arcLengths(path) {
  const s = [0];
  for (let i = 1; i < path.length; i++) {
    s.push(s[i - 1] + Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]));
  }
  const total = s[s.length - 1] + Math.hypot(
    path[0][0] - path[path.length - 1][0], path[0][1] - path[path.length - 1][1]
  );
  return { s, total };
}

// Builds one flat ribbon of the given width along `path[from..to]`, as a
// triangle list in the XY plane at z = 0 (the caller lifts the whole group).
function ribbonVertices(path, indices, width) {
  const half = width / 2;
  const normals = normalsClosed(path);
  const out = [];
  for (let k = 0; k + 1 < indices.length; k++) {
    const i = indices[k];
    const j = indices[k + 1];
    const [ax, ay] = path[i];
    const [bx, by] = path[j];
    const [anx, any] = normals[i];
    const [bnx, bny] = normals[j];
    const a0 = [ax + anx * half, ay + any * half, 0];
    const a1 = [ax - anx * half, ay - any * half, 0];
    const b0 = [bx + bnx * half, by + bny * half, 0];
    const b1 = [bx - bnx * half, by - bny * half, 0];
    out.push(...a0, ...a1, ...b0);
    out.push(...a1, ...b1, ...b0);
  }
  return out;
}

// Index ranges to draw, given arc-length intervals to skip (junction
// openings) or a dash pattern.
function solidRanges(s, total, skip) {
  const inSkip = (value) => skip.some(([s0, s1]) => {
    const v0 = value;
    const v1 = value + total;
    return (v0 >= s0 && v0 < s1) || (v1 >= s0 && v1 < s1);
  });
  const ranges = [];
  let current = [];
  for (let i = 0; i < s.length; i++) {
    if (inSkip(s[i])) {
      if (current.length > 1) ranges.push(current);
      current = [];
    } else {
      current.push(i);
    }
  }
  if (current.length > 1) ranges.push(current);
  return ranges;
}

function dashRanges(s, total, markM, gapM) {
  const pitch = markM + gapM;
  const ranges = [];
  let current = [];
  for (let i = 0; i < s.length; i++) {
    if (s[i] % pitch < markM) {
      current.push(i);
    } else {
      if (current.length > 1) ranges.push(current);
      current = [];
    }
  }
  if (current.length > 1) ranges.push(current);
  return ranges;
}

function ribbonMesh(path, ranges, width, material) {
  const vertices = [];
  ranges.forEach((indices) => vertices.push(...ribbonVertices(path, indices, width)));
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
  return new THREE.Mesh(geometry, material);
}

/**
 * The outer loop's three white lines, generated from the reference path so
 * they are exactly COURSE_GEOMETRY.lineWidthM wide and exactly
 * COURSE_GEOMETRY.laneWidthM apart. Returns a Group in ROS coordinates,
 * lifted just clear of the course texture plane (z = 0.01).
 *
 * MeshBasicMaterial (unlit), like the texture plane itself, so the lines
 * read the same under any lighting -- what the onboard camera feeds to
 * YOLOP/UFLD must not depend on the sun angle.
 */
export function createCourseLines() {
  const { centerPath, laneWidthM, lineWidthM, dash, innerGaps } = COURSE_GEOMETRY;
  const { s, total } = arcLengths(centerPath);
  const material = new THREE.MeshBasicMaterial({ color: LINE_COLOR, side: THREE.DoubleSide });

  const outerPath = offsetClosed(centerPath, laneWidthM);
  const innerPath = offsetClosed(centerPath, -laneWidthM);

  const group = new THREE.Group();
  group.add(ribbonMesh(outerPath, solidRanges(s, total, []), lineWidthM, material));
  group.add(ribbonMesh(innerPath, solidRanges(s, total, innerGaps), lineWidthM, material));
  group.add(ribbonMesh(centerPath, dashRanges(s, total, dash.markM, dash.gapM), lineWidthM, material));
  group.position.z = 0.02;
  return group;
}
```

**注意**: `offsetClosed` の符号は `course_geometry.js` の生成側（`build_course.py` の `sign_outer`）と一致していなければならない。Step 3 の検証でずれていたら、`createCourseLines()` の `outerPath` / `innerPath` の符号を入れ替えるのではなく、`build_course.py` が `sign_outer` を `course_geometry.js` に `outerSign` として書き出すよう直し、ここで読む（生成側を真とする）。

- [ ] **Step 2: `simulator.js` から呼ぶ**

`web_simulator/js/simulator.js` の import 行を更新:

```js
import { createCourseTexture, createCourseLines, COURSE_WIDTH_M, COURSE_DEPTH_M } from './course.js';
```

`rosRoot.add(course);` の直後に追加:

```js
// The outer loop's three white lines, drawn as geometry at the exact
// 15cm width / 3.5m lane spacing (js/course.js). The texture underneath
// has those three lines erased, so these are the only ones on the loop.
rosRoot.add(createCourseLines());
```

- [ ] **Step 3: ブラウザで確認**

サーバを起動して `http://localhost:8000/web_simulator/index.html` を開く。

Run（ページ内）:
```js
const { COURSE_GEOMETRY } = await import('/web_simulator/js/course_geometry.js');
JSON.stringify({ pts: COURSE_GEOMETRY.centerPath.length, len: COURSE_GEOMETRY.lengthM,
                 start: COURSE_GEOMETRY.startPose, gaps: COURSE_GEOMETRY.innerGaps })
```
Expected: `pts` 約 2470、`len` 約 247

コンソールにエラーが無いこと、外周ループに 3 本の白線が二重にならずに描かれていること、破線が破線として見えることをスクリーンショットで確認する。白線が背景の塗り潰し跡からはみ出していないことも見る。

- [ ] **Step 4: コミット**

```bash
git add web_simulator/js/course.js web_simulator/js/simulator.js
git commit -m "$(cat <<'EOF'
feat(web_simulator): draw the outer loop's white lines as exact geometry

15cm-wide ribbons generated from the reference path, replacing the texture's
own lines (which were 24cm at 1.5px and could not be made exact).

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 5: 縮尺関連定数の更新（`simulator.js`）

**Files:**
- Modify: `web_simulator/js/simulator.js`

**Interfaces:**
- Consumes: `COURSE_GEOMETRY.startPose`、`COURSE_GEOMETRY.laneWidthM`、Task 2 の診断出力にある world shift
- Produces: なし（既存定数の更新のみ）

- [ ] **Step 1: `COURSE_POSE` を更新**

`build_course.py` が出力した `world shift` を `COURSE_POSE` に反映する。仕様書の計算では:

```
COURSE_POSE_new = (0, -1.6) - 1.0680 * ((0, -1.6) - (13.22, 35.41)) = (14.119, 37.927)
```

ただし `build_course.py` は平滑化後のパスを `START_WORLD` に合わせる平行移動を別途かけているので、**ツールが表示した `world shift` を足した値**を使うこと。ツールの出力行:

```
world shift : <dx>, <dy>  -> COURSE_POSE must move by the same amount
```

`simulator.js`:

```js
const COURSE_POSE = { x: 14.119 + SHIFT_X, y: 37.927 + SHIFT_Y, z: 0.01, roll: 0, pitch: 0, yaw: Math.PI / 2 };
```

は使わず、**ツールが出した最終値をそのまま定数として書く**（計算式をコードに残さない）。コメントで根拠を書く:

```js
// Course layout plane. Position recomputed for the 106.80m scale so the
// texture's own outer loop lands on the generated 3.5m-lane geometry, with
// the vehicle's spawn point held at the world origin anchor (0, -1.6) --
// see tools/build_course.py's "world shift" diagnostic and
// docs/superpowers/specs/2026-09-21-course-geometry-and-collision-design.md.
const COURSE_POSE = { x: <tool value>, y: <tool value>, z: 0.01, roll: 0, pitch: 0, yaw: Math.PI / 2 };
```

- [ ] **Step 2: `SIM_LANE_WIDTH` を更新**

```js
// lane_width: this course's center line <-> boundary line distance. Now an
// exact property of the generated geometry rather than a measurement of the
// texture (it was 3.1 when the traced course averaged 3.27m).
const SIM_LANE_WIDTH = COURSE_GEOMETRY.laneWidthM;
```

import に `COURSE_GEOMETRY` を追加:

```js
import { COURSE_GEOMETRY } from './course_geometry.js';
```

- [ ] **Step 3: `SIM_START_POSE` を幾何から取る**

```js
// Start pose on the center white line of the outer loop (the lap-1 method
// drives on top of it). Taken from the generated geometry rather than
// hand-measured, so it stays on the line whenever the course is rebuilt.
const SIM_START_POSE = COURSE_GEOMETRY.startPose;
```

`physics.x = SIM_START_POSE.x; physics.y = SIM_START_POSE.y; physics.yaw = SIM_START_POSE.yaw;` の代入箇所はそのまま動く（`startPose` が `{x, y, yaw}` なので）。

- [ ] **Step 4: ブラウザで確認**

ページを再読み込みし、Run（ページ内）:
```js
const { COURSE_GEOMETRY } = await import('/web_simulator/js/course_geometry.js');
const p = window.__sim.physics;
const path = COURSE_GEOMETRY.centerPath;
let best = Infinity;
for (const q of path) best = Math.min(best, Math.hypot(q[0] - p.x, q[1] - p.y));
JSON.stringify({ spawn: [p.x, p.y, p.yaw], distanceToCentreLine: best })
```
Expected: `distanceToCentreLine` < 0.05

スクリーンショットで、車両が中央線の上に乗っていること、背景 PNG の外周ループと生成した白線がずれていないことを確認する。

- [ ] **Step 5: コミット**

```bash
git add web_simulator/js/simulator.js
git commit -m "$(cat <<'EOF'
feat(web_simulator): rescale the course to real dimensions (3.5m lanes)

COURSE_POSE moves with the 106.80m plane; SIM_LANE_WIDTH and SIM_START_POSE
now come from the generated geometry instead of hand-measured constants.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 6: MyLaps の再配置と衝突円

**Files:**
- Modify: `web_simulator/js/course_props.js`

**Interfaces:**
- Consumes: `COURSE_GEOMETRY.centerPath`
- Produces:
  - `MYLAPS_POSE: {x, y, z, roll, pitch, yaw}`（新縮尺での再計算値）
  - `MYLAPS_COLLIDERS: Array<{x: number, y: number, r: number}>` — モデルローカル [m]
  - `worldColliders(pose, colliders): Array<{x, y, r}>` — 世界座標に変換したもの
  - `addMyLapsGantry(parent, setPose)` の戻り値を `THREE.Group`（ルート）に変更

- [ ] **Step 1: 新しい設置座標を算出**

Run:
```bash
cd web_simulator && python3 - <<'EOF'
import json, math, re
s = open('js/course_geometry.js').read()
g = json.loads(re.search(r'COURSE_GEOMETRY = (\{.*?\});', s, re.S).group(1))
path = g['centerPath']; n = len(path)
S = [0.0]
for i in range(1, n):
    S.append(S[-1] + math.dist(path[i], path[i-1]))
def at(s_):
    s_ %= S[-1]
    for i in range(1, n):
        if S[i] >= s_:
            t = (s_ - S[i-1]) / (S[i] - S[i-1])
            return (path[i-1][0] + t*(path[i][0]-path[i-1][0]), path[i-1][1] + t*(path[i][1]-path[i-1][1]))
def hdg(s_, w=6.0):
    a, b = at(s_-w/2), at(s_+w/2)
    return math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))
# 2 コーナーの出口: 旧縮尺で s=111.0 -> 新縮尺では 111.0 * (新全長 / 231.2)
for s_ in range(70, 140):
    print(s_, [round(v, 2) for v in at(s_)], round(hdg(s_), 1))
EOF
```

出力から、進行方向の方位が 180° に落ち着く点（2 コーナー出口）を読み取り、そこから +25 m の座標と方位を求める。仕様書の基準では旧縮尺 s=111.0 が出口、+25 m で s=136.0。新縮尺では全長が 231.2 → 約 247 m なので、同じ場所は s ≈ 111.0 × 1.0680 = 118.5、目標は s ≈ 118.5 + 25 = 143.5。**弧長のスケールではなく方位プロファイルから出口を読み直すこと**（+25 m は実寸なのでスケールしない）。

- [ ] **Step 2: `MYLAPS_POSE` を更新**

```js
// Placement: on the course centre line (js/course_geometry.js centerPath),
// 25m past the exit of the second corner, measured along the centre line in
// the driving direction.
//
//   corner 2 (the long left-hander onto the top straight) exits at
//   s=<tool value>m; +25m along the centre line lands on s=<tool value>m.
//
// yaw: the arch spans the track and its LED panel (model +y) faces oncoming
// traffic. The top straight is driven in -x, so model +y must point to world
// +x, i.e. yaw = track heading + 90deg = 180 + 90 = -90deg.
//
// z is lifted just clear of the course texture plane (COURSE_POSE.z = 0.01)
// so the base bars don't z-fight with it.
export const MYLAPS_POSE = { x: <new>, y: <new>, z: 0.02, roll: 0, pitch: 0, yaw: -Math.PI / 2 };
```

- [ ] **Step 3: 衝突円を追加**

```js
// Collision footprint, as circles in the model's own XY plane (metres,
// relative to MYLAPS_POSE). Listed explicitly rather than derived from the
// .obj's group bounding boxes, so swapping the model out cannot silently
// change what the vehicle can hit. Values are the group extents of
// MyLaps.obj scaled by MYLAPS_SCALE:
//   Post_L/R   x = -/+23cm, y = -3.2cm, radius 1.9cm
//   Cone1/2/3  x = -31 / 0 / +31cm, y = 55cm, base radius 15cm
export const MYLAPS_COLLIDERS = [
  { x: -0.23, y: -0.032, r: 0.019 },
  { x: 0.23, y: -0.032, r: 0.019 },
  { x: -0.31, y: 0.55, r: 0.15 },
  { x: 0.0, y: 0.55, r: 0.15 },
  { x: 0.31, y: 0.55, r: 0.15 },
];

/**
 * Model-local collider circles placed into the world by a ROS pose.
 * @param {{x:number, y:number, yaw:number}} pose
 * @param {Array<{x:number, y:number, r:number}>} colliders
 * @returns {Array<{x:number, y:number, r:number}>}
 */
export function worldColliders(pose, colliders) {
  const c = Math.cos(pose.yaw);
  const s = Math.sin(pose.yaw);
  return colliders.map((o) => ({
    x: pose.x + c * o.x - s * o.y,
    y: pose.y + s * o.x + c * o.y,
    r: o.r,
  }));
}
```

- [ ] **Step 4: プロップのルートを返すようにする**

`addMyLapsGantry` を変更し、読み込み完了を待たずに使えるルート Group を即座に返す:

```js
export function addMyLapsGantry(parent, setPose) {
  const root = new THREE.Group();
  setPose(root, MYLAPS_POSE);
  parent.add(root);
  loadObj(MODEL_DIR + 'MyLaps.obj', MODEL_DIR + 'MyLaps.mtl')
    .then((gantry) => {
      gantry.scale.setScalar(MYLAPS_SCALE);
      root.add(gantry);
    })
    .catch((err) => console.error('Failed to load models/MyLaps.obj', err));
  return root;
}
```

- [ ] **Step 5: ブラウザで確認**

Run（ページ内）:
```js
const { MYLAPS_POSE, MYLAPS_COLLIDERS, worldColliders } = await import('/web_simulator/js/course_props.js');
const { COURSE_GEOMETRY } = await import('/web_simulator/js/course_geometry.js');
let best = Infinity;
for (const q of COURSE_GEOMETRY.centerPath) best = Math.min(best, Math.hypot(q[0] - MYLAPS_POSE.x, q[1] - MYLAPS_POSE.y));
JSON.stringify({ pose: MYLAPS_POSE, distanceToCentreLine: best, colliders: worldColliders(MYLAPS_POSE, MYLAPS_COLLIDERS) })
```
Expected: `distanceToCentreLine` < 0.05、`colliders` が 5 個

- [ ] **Step 6: コミット**

```bash
git add web_simulator/js/course_props.js
git commit -m "$(cat <<'EOF'
feat(web_simulator): reposition MyLaps for the new scale and give it colliders

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 7: 当たり判定のコア（`collision.js`）と検証スクリプト

**Files:**
- Create: `web_simulator/js/collision.js`
- Create: `web_simulator/tools/verify_course.js`

**Interfaces:**
- Consumes: `COURSE_GEOMETRY`
- Produces:
  - `VEHICLE_COLLIDERS: Array<{x: number, r: number}>`
  - `resolveCollisions(pose, obstacles, vehicleColliders): {dx, dy, headOn, maxPenetration}` — `pose` は `{x, y, yaw}`
  - `class PathTracker { constructor(path); update(x, y): {offset, s, index} }`
  - `class DepartureMonitor { constructor(opts); update(offset): {outside, count, justLeft} }`
  - `runChecks(): Promise<{pass: boolean, results: Array<{name, pass, detail}>}>`（`verify_course.js`）

- [ ] **Step 1: `collision.js` を実装**

```js
// 2D collision helpers for the simulator. Pure geometry -- no three.js, no
// scene access -- so the same functions back both the runtime checks in
// js/simulator.js and the verification script in tools/verify_course.js.

import { COURSE_GEOMETRY } from './course_geometry.js';

// Vehicle footprint: two circles along the body axis, in base_link metres.
// Covers roughly 1.6m x 0.8m -- the xacro body runs from the caster at
// x = -0.76 to about x = +0.7, and the 0.40 radius matches
// lane_navigator.js's RACELINE_PARAMS.vehicleHalfWidth.
export const VEHICLE_HALF_WIDTH = 0.40;
export const VEHICLE_COLLIDERS = [
  { x: 0.30, r: VEHICLE_HALF_WIDTH },
  { x: -0.50, r: VEHICLE_HALF_WIDTH },
];

/**
 * Pushes the vehicle out of any obstacle it overlaps.
 *
 * Returns the positional correction to apply, whether the contact is close
 * to head-on (the caller zeroes forward speed then -- VehiclePhysics carries
 * only a scalar forward speed, so a normal/tangential velocity split is not
 * meaningful), and the deepest penetration seen.
 *
 * @param {{x:number, y:number, yaw:number}} pose
 * @param {Array<{x:number, y:number, r:number}>} obstacles world-frame circles
 * @param {Array<{x:number, r:number}>} vehicleColliders base_link circles
 */
export function resolveCollisions(pose, obstacles, vehicleColliders = VEHICLE_COLLIDERS) {
  const c = Math.cos(pose.yaw);
  const s = Math.sin(pose.yaw);
  let dx = 0;
  let dy = 0;
  let headOn = false;
  let maxPenetration = 0;

  for (const vc of vehicleColliders) {
    // Re-evaluate the circle's world position against corrections already
    // accumulated this step, so two obstacles cannot cancel each other out.
    const wx = pose.x + c * vc.x + dx;
    const wy = pose.y + s * vc.x + dy;
    for (const o of obstacles) {
      const ox = wx - o.x;
      const oy = wy - o.y;
      const dist = Math.hypot(ox, oy);
      const penetration = vc.r + o.r - dist;
      if (penetration <= 0) continue;
      const nx = dist > 1e-9 ? ox / dist : 1;
      const ny = dist > 1e-9 ? oy / dist : 0;
      dx += nx * penetration;
      dy += ny * penetration;
      maxPenetration = Math.max(maxPenetration, penetration);
      if (c * nx + s * ny < -0.5) headOn = true;
    }
  }
  return { dx, dy, headOn, maxPenetration };
}

/**
 * Tracks where the vehicle is along the closed reference path and how far it
 * sits to the side of it. Searches only near the previous index (the path is
 * 0.10m-sampled, so +/-400 points is 40m of travel between frames) instead of
 * scanning all ~2470 points every frame.
 */
export class PathTracker {
  constructor(path = COURSE_GEOMETRY.centerPath, window = 400) {
    this.path = path;
    this.window = window;
    this.index = 0;
    this.s = [0];
    for (let i = 1; i < path.length; i++) {
      this.s.push(this.s[i - 1] + Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]));
    }
  }

  reset(index = 0) {
    this.index = index;
  }

  /** @returns {{offset:number, s:number, index:number}} offset is +left of the path */
  update(x, y) {
    const n = this.path.length;
    let best = Infinity;
    let bi = this.index;
    for (let k = -this.window; k <= this.window; k++) {
      const i = ((this.index + k) % n + n) % n;
      const d = (this.path[i][0] - x) ** 2 + (this.path[i][1] - y) ** 2;
      if (d < best) {
        best = d;
        bi = i;
      }
    }
    this.index = bi;
    const a = this.path[bi];
    const b = this.path[(bi + 1) % n];
    const tx = b[0] - a[0];
    const ty = b[1] - a[1];
    const len = Math.hypot(tx, ty) || 1e-12;
    const offset = (-(ty / len)) * (x - a[0]) + (tx / len) * (y - a[1]);
    return { offset, s: this.s[bi], index: bi };
  }
}

/**
 * Counts course departures with hysteresis: one count when the vehicle first
 * crosses `leaveM`, and no further counts until it comes back inside
 * `returnM`. Driving is never blocked -- this only reports.
 */
export class DepartureMonitor {
  constructor({ leaveM, returnM } = {}) {
    const lane = COURSE_GEOMETRY.laneWidthM;
    const line = COURSE_GEOMETRY.lineWidthM;
    // Any part of the vehicle past the outer edge of the boundary line.
    this.leaveM = leaveM !== undefined ? leaveM : lane + line / 2 - VEHICLE_HALF_WIDTH;
    this.returnM = returnM !== undefined ? returnM : this.leaveM - 0.175;
    this.count = 0;
    this.outside = false;
  }

  reset() {
    this.count = 0;
    this.outside = false;
  }

  /** @returns {{outside:boolean, count:number, justLeft:boolean}} */
  update(offset) {
    const magnitude = Math.abs(offset);
    let justLeft = false;
    if (!this.outside && magnitude > this.leaveM) {
      this.outside = true;
      this.count += 1;
      justLeft = true;
    } else if (this.outside && magnitude < this.returnM) {
      this.outside = false;
    }
    return { outside: this.outside, count: this.count, justLeft };
  }
}
```

`returnM` の既定値は `leaveM - 0.175 = 3.0`（仕様書の 3.0 m）。

- [ ] **Step 2: 検証スクリプトを実装**

`web_simulator/tools/verify_course.js`:

```js
// Browser-side verification for the generated course and the collision
// helpers. There is no node on this host, so this is run by importing it in
// the page and calling runChecks():
//
//   const m = await import('/web_simulator/tools/verify_course.js');
//   await m.runChecks();
//
// Checks 1-6 of the design doc's 検証方針 section. Checks 7-8 (autonomous
// laps) are driven by hand, not from here.

import { COURSE_GEOMETRY } from '../js/course_geometry.js';
import { COURSE_LINES } from '../js/course_lines.js';
import {
  DepartureMonitor, PathTracker, VEHICLE_COLLIDERS, resolveCollisions,
} from '../js/collision.js';
import { MYLAPS_COLLIDERS, MYLAPS_POSE, worldColliders } from '../js/course_props.js';

function nearestDistance(point, path) {
  let best = Infinity;
  for (let i = 0; i < path.length; i++) {
    const a = path[i];
    const b = path[(i + 1) % path.length];
    const abx = b[0] - a[0];
    const aby = b[1] - a[1];
    const l2 = Math.max(abx * abx + aby * aby, 1e-12);
    const t = Math.max(0, Math.min(1, ((point[0] - a[0]) * abx + (point[1] - a[1]) * aby) / l2));
    const d = Math.hypot(point[0] - a[0] - t * abx, point[1] - a[1] - t * aby);
    if (d < best) best = d;
  }
  return best;
}

export async function runChecks() {
  const results = [];
  const check = (name, pass, detail) => results.push({ name, pass, detail });
  const path = COURSE_GEOMETRY.centerPath;

  // 1. 寸法
  for (const side of ['outer', 'inner']) {
    let min = Infinity;
    let max = -Infinity;
    for (const p of COURSE_LINES[side]) {
      const d = nearestDistance(p, path);
      min = Math.min(min, d);
      max = Math.max(max, d);
    }
    check(`lane width (${side})`, Math.abs(min - 3.5) < 0.001 && Math.abs(max - 3.5) < 0.001,
      `min ${min.toFixed(4)} max ${max.toFixed(4)}`);
  }
  check('line width constant', COURSE_GEOMETRY.lineWidthM === 0.15, `${COURSE_GEOMETRY.lineWidthM}`);

  // 3. スポーン
  const sim = window.__sim;
  if (sim) {
    const d = nearestDistance([sim.physics.x, sim.physics.y], path);
    check('spawn on the centre line', d < 0.05, `${d.toFixed(4)} m`);
  }

  // 4. 理想検出との一致
  if (sim && sim.idealDetector) {
    const pose = { x: COURSE_GEOMETRY.startPose.x, y: COURSE_GEOMETRY.startPose.y, yaw: COURSE_GEOMETRY.startPose.yaw };
    const tracker = new PathTracker();
    const { offset } = tracker.update(pose.x, pose.y);
    check('ideal detector frame matches geometry', Math.abs(offset) < 0.01, `offset ${offset.toFixed(4)} m`);
  }

  // 5. 障害物
  const obstacles = worldColliders(MYLAPS_POSE, MYLAPS_COLLIDERS);
  const headOnPose = { x: MYLAPS_POSE.x + 0.5, y: MYLAPS_POSE.y, yaw: Math.PI };
  const r = resolveCollisions(headOnPose, obstacles, VEHICLE_COLLIDERS);
  check('gate blocks a head-on approach', r.maxPenetration > 0 && r.headOn,
    `penetration ${r.maxPenetration.toFixed(4)} m, headOn ${r.headOn}`);
  const clearPose = { x: MYLAPS_POSE.x + 20, y: MYLAPS_POSE.y, yaw: Math.PI };
  const clear = resolveCollisions(clearPose, obstacles, VEHICLE_COLLIDERS);
  check('no contact when clear', clear.maxPenetration === 0, `penetration ${clear.maxPenetration}`);

  // 6. 逸脱判定
  const monitor = new DepartureMonitor();
  const seen = [];
  for (const offset of [0, 2.0, 3.0, 3.1, 3.2, 3.3, 2.9, 3.2]) {
    seen.push(monitor.update(offset).count);
  }
  check('departure counts once per excursion', JSON.stringify(seen) === JSON.stringify([0, 0, 0, 0, 1, 1, 1, 2]),
    JSON.stringify(seen));

  const pass = results.every((x) => x.pass);
  console.table(results);
  return { pass, results };
}
```

- [ ] **Step 3: ブラウザで検証スクリプトを走らせる**

Run（ページ内）:
```js
const m = await import('/web_simulator/tools/verify_course.js');
JSON.stringify(await m.runChecks(), null, 1)
```
Expected: `pass: true`。落ちた項目があれば、その `detail` を見て該当タスクに戻る

- [ ] **Step 4: コミット**

```bash
git add web_simulator/js/collision.js web_simulator/tools/verify_course.js
git commit -m "$(cat <<'EOF'
feat(web_simulator): add 2D collision helpers and a browser verification script

No node on this host, so the checks run by importing verify_course.js in the
page rather than as a unit test suite.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 8: 障害物衝突をシミュレータに配線

**Files:**
- Modify: `web_simulator/js/simulator.js`

**Interfaces:**
- Consumes: `resolveCollisions`, `VEHICLE_COLLIDERS`（Task 7）、`MYLAPS_COLLIDERS`, `MYLAPS_POSE`, `worldColliders`（Task 6）
- Produces: `window.__sim.obstacles`, `window.__sim.mylapsRoot`（検証で一時的に無効化するため）

- [ ] **Step 1: import と障害物リストを追加**

```js
import { resolveCollisions, VEHICLE_COLLIDERS } from './collision.js';
import { addMyLapsGantry, MYLAPS_COLLIDERS, MYLAPS_POSE, worldColliders } from './course_props.js';
```

`addMyLapsGantry(rosRoot, setPose);` を置き換える:

```js
// MyLaps timing gantry on the centre line, 25m past the second corner
// (js/course_props.js). Its posts and cones are solid -- the vehicle cannot
// pass between the 0.46m posts, which is intentional: it is the obstacle the
// planned YOLO cone detector will have to steer around.
const mylapsRoot = addMyLapsGantry(rosRoot, setPose);
const obstacles = worldColliders(MYLAPS_POSE, MYLAPS_COLLIDERS);
```

- [ ] **Step 2: 物理ステップの後に衝突解決を入れる**

`physics.step(...)` / `physics.stepAutonomous(...)` を呼んでいる箇所の直後（両方を通る 1 箇所にまとめられるなら、そこ）に:

```js
  // Obstacle collision: push the vehicle back out of anything it overlaps.
  // Applied after integration as a position correction, so VehiclePhysics
  // itself stays a clean kinematic model. Sliding falls out of the
  // correction (only the component along the contact normal is cancelled);
  // a near head-on contact additionally kills forward speed.
  if (!mylapsRoot.userData.collisionDisabled) {
    const hit = resolveCollisions(physics, obstacles, VEHICLE_COLLIDERS);
    if (hit.maxPenetration > 0) {
      physics.x += hit.dx;
      physics.y += hit.dy;
      if (hit.headOn && physics.v > 0) physics.v = 0;
    }
    contactActive = hit.maxPenetration > 0;
  } else {
    contactActive = false;
  }
```

`let contactActive = false;` をモジュールスコープに置く。

- [ ] **Step 3: デバッグオブジェクトに露出**

`window.__sim` のリテラルに追加:

```js
  obstacles, mylapsRoot,
```

- [ ] **Step 4: ブラウザで確認（貫入が起きないこと）**

Run（ページ内）:
```js
const p = window.__sim.physics;
const { MYLAPS_POSE } = await import('/web_simulator/js/course_props.js');
const { resolveCollisions, VEHICLE_COLLIDERS } = await import('/web_simulator/js/collision.js');
p.x = MYLAPS_POSE.x + 3.0; p.y = MYLAPS_POSE.y; p.yaw = Math.PI; p.v = 1.5;
let worst = 0;
for (let i = 0; i < 400; i++) {
  await new Promise((r) => requestAnimationFrame(r));
  const h = resolveCollisions(p, window.__sim.obstacles, VEHICLE_COLLIDERS);
  worst = Math.max(worst, h.maxPenetration);
}
JSON.stringify({ worstPenetration: worst, finalX: p.x, v: p.v })
```
Expected: `worstPenetration` < 0.001、`finalX` がゲート手前（`MYLAPS_POSE.x + 0.6` 付近）で止まっている、`v` が 0

- [ ] **Step 5: コミット**

```bash
git add web_simulator/js/simulator.js
git commit -m "$(cat <<'EOF'
feat(web_simulator): make the MyLaps gantry and cones solid obstacles

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 9: コース逸脱の検出と HUD

**Files:**
- Modify: `web_simulator/js/simulator.js`
- Modify: `web_simulator/index.html`

**Interfaces:**
- Consumes: `PathTracker`, `DepartureMonitor`（Task 7）
- Produces: `window.__sim.pathTracker`, `window.__sim.departureMonitor`

- [ ] **Step 1: HUD 要素を追加**

`index.html` の、速度表示（`速度: ... m/s`）を含むブロックの直後に:

```html
<div id="course-status" style="margin-top:6px;">
  <span id="departure-state" style="font-weight:bold;">コース内</span>
  <span style="opacity:0.7;"> / 逸脱 </span><span id="departure-count">0</span><span style="opacity:0.7;"> 回</span>
  <span id="contact-state" style="display:none; color:#ff6b6b; font-weight:bold;"> ／ 接触中</span>
</div>
```

- [ ] **Step 2: 逸脱判定を配線**

`simulator.js` の `collision.js` からの import に `PathTracker` と `DepartureMonitor` を足す:

```js
import { resolveCollisions, VEHICLE_COLLIDERS, PathTracker, DepartureMonitor } from './collision.js';
```

初期化部に:

```js
// Course departure: reported, never blocked. The vehicle can leave the
// track and drive back on; the HUD counts each excursion once.
const pathTracker = new PathTracker();
const departureMonitor = new DepartureMonitor();
const departureStateEl = document.getElementById('departure-state');
const departureCountEl = document.getElementById('departure-count');
const contactStateEl = document.getElementById('contact-state');
```

Task 8 で入れた衝突処理の直後に:

```js
  const { offset } = pathTracker.update(physics.x, physics.y);
  const departure = departureMonitor.update(offset);
  departureStateEl.textContent = departure.outside ? '逸脱中' : 'コース内';
  departureStateEl.style.color = departure.outside ? '#ff6b6b' : '#8fd18f';
  departureCountEl.textContent = String(departure.count);
  contactStateEl.style.display = contactActive ? 'inline' : 'none';
```

`resetNavigation()` に追加:

```js
  pathTracker.reset();
  departureMonitor.reset();
```

`window.__sim` に `pathTracker, departureMonitor,` を追加。

- [ ] **Step 3: ブラウザで確認**

Run（ページ内）:
```js
const { COURSE_GEOMETRY } = await import('/web_simulator/js/course_geometry.js');
const p = window.__sim.physics;
const m = window.__sim.departureMonitor;
m.reset();
const start = COURSE_GEOMETRY.startPose;
const seen = [];
for (const dy of [0, 2.0, 3.0, 3.3, 2.5, 3.3]) {
  p.x = start.x; p.y = start.y + dy; p.v = 0;
  await new Promise((r) => requestAnimationFrame(r));
  seen.push(m.count);
}
JSON.stringify({ counts: seen, leaveM: m.leaveM, returnM: m.returnM })
```
Expected: `leaveM` 3.175、`returnM` 3.0、`counts` が `[0,0,0,1,1,2]`

HUD の「コース内 / 逸脱中」表示と回数がスクリーンショットで確認できること。

- [ ] **Step 4: コミット**

```bash
git add web_simulator/js/simulator.js web_simulator/index.html
git commit -m "$(cat <<'EOF'
feat(web_simulator): detect and report course departures in the HUD

Warning only -- driving continues, so the recovery behaviour after leaving
the track stays observable.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 10: README と総合検証

**Files:**
- Modify: `web_simulator/README.md`

**Interfaces:**
- Consumes: すべて
- Produces: なし

- [ ] **Step 1: 全ユニットテストを走らせる**

Run: `cd web_simulator/tools && python3 -m unittest test_build_course -v`
Expected: PASS（29 tests）

- [ ] **Step 2: ブラウザ検証スクリプトを走らせる**

Run（ページ内）:
```js
const m = await import('/web_simulator/tools/verify_course.js');
JSON.stringify(await m.runChecks(), null, 1)
```
Expected: `pass: true`

- [ ] **Step 3: 自動運転をゲート無効で 1 周（仕様書 検証項目 7）**

Run（ページ内）:
```js
window.__sim.mylapsRoot.visible = false;
window.__sim.mylapsRoot.userData.collisionDisabled = true;
window.__sim.resetNavigation();
'gate disabled -- now enable 自動運転 in the HUD and watch a mapping lap + a racing lap'
```
「自動運転」トグルを ON にして、マッピング 1 周 → レーシングライン 1 周が完走することを確認する。完走しない場合は `lane_navigator.js` のどのパラメータで詰まったかを記録し、縮尺起因かを切り分ける（仕様書は「変更不要の見込み」としている）。

- [ ] **Step 4: ゲートを戻して停止を確認（仕様書 検証項目 8）**

Run（ページ内）:
```js
window.__sim.mylapsRoot.visible = true;
window.__sim.mylapsRoot.userData.collisionDisabled = false;
window.__sim.resetNavigation();
'gate enabled -- autonomous driving should now stop at the gate'
```
ゲート手前で停止することを確認する。これは期待挙動であって失敗ではない。

- [ ] **Step 5: README を更新**

`web_simulator/README.md` に節を追加:

```markdown
## コースの実寸

外周ループは実寸で作られています。

| 項目 | 値 |
|---|---|
| 車線幅（中央線↔境界線） | 3.5 m |
| 白線幅 | 15 cm |
| 中央線の破線 | 2.8 m / 2.6 m |
| 外周ループ全長（中央線） | 約 247 m |
| コース板 | 106.80 m × 85.42 m |

外周ループの 3 本の白線は、テクスチャではなく `js/course_geometry.js` の基準パスから
生成したリボンメッシュです（テクスチャ側は同じ線を消してあります）。内側の交差点・
横断歩道・駐車枠・芝生島は従来どおりテクスチャで、同じ縮尺に載っています。

### コース定義の再生成

```
python3 web_simulator/tools/build_course.py
```

`png/shihou_cource_unity.png` から以下を生成します。手で編集しないでください。

- `js/course_geometry.js` — 基準パスと寸法定数
- `js/course_lines.js` — 3 本線の点列（理想検出モードの入力）
- `png/shihou_cource_base.png` — 外周 3 本線を消した背景テクスチャ

ユニットテスト: `cd web_simulator/tools && python3 -m unittest test_build_course`

## 当たり判定

- **障害物（MyLaps ゲートの支柱とコーン）**: 物理的に阻止します。車体は 2 円（半径 0.40 m）で
  近似し、貫入した分だけ押し戻します。正対して当たった場合は前進速度を 0 にします。
  ゲートの支柱間は 0.46 m で車幅 0.8 m より狭いため**通り抜けられません**。これは意図的で、
  今後導入するコーン検知 YOLO の回避行動の対象になります。
- **コース逸脱**: 走行は止めず、HUD に「逸脱中」表示と累計回数を出すだけです。
  車体の一部が境界線の外縁を越えた時点（基準パスから 3.175 m）で 1 回と数え、
  3.0 m 以内に戻るまで次を数えません。ROS トピックは発行しません。

ブラウザ上での検証: ページを開いてコンソールで

```js
const m = await import('/web_simulator/tools/verify_course.js');
await m.runChecks();
```
```

既存の「ディレクトリ構成」節にある `tools/extract_course_lines.py` の記述を `tools/build_course.py` に直し、`js/course_geometry.js` / `js/collision.js` / `png/shihou_cource_base.png` / `tools/verify_course.js` を足す。

- [ ] **Step 6: コミット**

```bash
git add web_simulator/README.md
git commit -m "$(cat <<'EOF'
docs(web_simulator): document the course's real dimensions and collision

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
EOF
)"
```

---

## Self-Review

**Spec coverage:**

| 仕様書の節 | 担当タスク |
|---|---|
| 1. 縮尺 | Task 2（`SCALE_K` / `COURSE_WIDTH_M`）, Task 4（`course.js`）, Task 5（`COURSE_POSE`） |
| 2. 基準パスの平滑化 | Task 1（`lowpass_closed`）, Task 2（`build_geometry`） |
| 3. `course_geometry.js` | Task 2 |
| 4. ビルドツール | Task 1, 2, 3 |
| 5. 白線のリボン描画 | Task 4 |
| 6. 障害物の当たり判定 | Task 6（衝突円）, Task 7（`resolveCollisions`）, Task 8（配線） |
| 7. コース逸脱 | Task 7（`PathTracker` / `DepartureMonitor`）, Task 9（配線 + HUD） |
| 8. 既存の値の更新 | Task 4, 5, 6。`lane_navigator.js` は Task 10 Step 3 で実走確認 |
| 9. 検証方針 1〜6 | Task 7 の `verify_course.js` |
| 9. 検証方針 7〜8 | Task 10 Step 3, 4 |
| 決定事項（ゲートは通過不能） | Task 8 のコメント, Task 10 の README |

**既知のリスク（実装中に判明したら止めて報告すること）:**

1. **Task 2 の `inner_gaps` 算出**が、レイインデックス基準の欠損を弧長に変換する近似になっている。`test_inner_gaps_are_preserved` が 3 区間を見つけられない、あるいは区間が実際の接続口とずれる場合は、抽出インデックス → 基準パス弧長の対応を、抽出点を基準パスに投影して求める実装に差し替える
2. **Task 4 の `offsetClosed` の符号**が生成側とずれる可能性がある。Step 1 の注意書きのとおり、生成側（`build_course.py`）が `outerSign` を書き出す方向で直す
3. **Task 5 の `COURSE_POSE`** はツールの `world shift` 診断に依存する。ツールが `START_WORLD` へのアンカリングで平行移動を入れるため、仕様書の (14.119, 37.927) をそのまま使うと背景がずれる。必ずツールの出力値を使う
4. **Task 10 Step 3** で自動運転が完走しない場合、原因が縮尺かどうかの切り分けが必要。`lane_navigator.js` のパラメータ変更は仕様書の想定外なので、必要なら仕様書に追記してから直す
