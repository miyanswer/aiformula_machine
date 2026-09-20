"""build_course.py の幾何コアのユニットテスト (python3 -m unittest で実行)."""

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from build_course import (  # noqa: E402
    DASH_GAP_M, DASH_MARK_M, build_geometry, curvature_closed, dashed_line, fill_closed,
    gap_intervals, gapped_line, lowpass_closed, nearest_distance, normals_closed,
    offset_closed, resample_closed,
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

    def test_start_pose_yaw_matches_the_path_tangent(self):
        path = np.array(self.geom["centerPath"])
        tangent = path[1] - path[-1]
        expected_yaw = math.atan2(tangent[1], tangent[0])
        self.assertAlmostEqual(self.geom["startPose"]["yaw"], expected_yaw, delta=1e-3)

    def test_start_pose_is_the_world_origin_anchor(self):
        self.assertAlmostEqual(self.geom["startPose"]["x"], 0.0, delta=0.05)
        self.assertAlmostEqual(self.geom["startPose"]["y"], -1.6, delta=0.05)

    def test_inner_gaps_are_preserved(self):
        self.assertGreaterEqual(len(self.geom["innerGaps"]), 3)
        for s0, s1 in self.geom["innerGaps"]:
            self.assertGreater(s1 - s0, 1.5)

    def test_outer_sign_is_plus_or_minus_one(self):
        self.assertIn(self.geom["outerSign"], (1.0, -1.0))

    def test_outer_sign_points_toward_the_traced_outer_line(self):
        path = np.array(self.geom["centerPath"])
        sign = self.geom["outerSign"]
        traced_outer = np.array([p for p in self.geom["_traced"]["outer"] if p])
        d_correct = nearest_distance(traced_outer, offset_closed(path, sign * 3.5))
        d_flipped = nearest_distance(traced_outer, offset_closed(path, -sign * 3.5))
        self.assertLess(float(d_correct.mean()), float(d_flipped.mean()))


if __name__ == "__main__":
    unittest.main()
