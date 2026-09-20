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
