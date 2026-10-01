"""motor_math (速度指令 -> RPM -> CAN, 加減速制限) の単体テスト. ROS 不要:
    python3 -m pytest control/motor_controller/test/test_motor_math.py
"""

import math
import os
import struct
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from motor_controller.motor_math import rate_limit, rate_limit_linear, to_can_data, to_ref_rpm  # noqa: E402

D, T = 0.254, 0.60  # config/wheel.yaml


def test_straight_rpm_matches_wheel_speed():
    """1 m/s 直進 = 左右とも 60 / (pi * D) rpm (odometry_publisher/wheel.hpp の逆変換)."""
    left, right = to_ref_rpm(1.0, 0.0, D, T, 1.0)
    assert left == pytest.approx(60.0 / (math.pi * D)) and right == pytest.approx(left)


def test_turn_left_makes_right_wheel_faster():
    left, right = to_ref_rpm(1.0, 0.5, D, T, 1.0)
    assert right > left > 0.0
    assert (right - left) == pytest.approx(2 * (T / D) * 0.5 * 60 / (2 * math.pi))


def test_in_place_rotation_is_prevented():
    assert to_ref_rpm(0.1, 2.0, D, T, 1.0) == (0.0, 0.0)  # 左右が逆符号になる指令は止める


def test_can_data_layout_is_right_then_left_int32_le():
    data = to_can_data(1.0, 0.3, D, T, 1.0)
    left, right = to_ref_rpm(1.0, 0.3, D, T, 1.0)
    r, l = struct.unpack('<ii', bytes(data))
    assert (r, l) == (round(right), round(left)) and len(data) == 8


def test_linear_rate_limit_uses_decel_toward_zero_and_accel_away():
    assert rate_limit_linear(0.0, 2.0, 0.1, 2.2, 1.5) == pytest.approx(0.22)      # 加速 2.2
    assert rate_limit_linear(1.0, 0.0, 0.1, 2.2, 1.5) == pytest.approx(0.85)      # 減速 1.5
    assert rate_limit_linear(1.0, -1.0, 0.1, 2.2, 1.5) == pytest.approx(0.85)     # 前後反転は一度 0 へ減速
    assert rate_limit_linear(0.1, 0.0, 0.1, 2.2, 1.5) == 0.0                        # 行き過ぎない


def test_rate_limit():
    assert rate_limit(0.0, 1.0, 0.2) == pytest.approx(0.2)
    assert rate_limit(0.0, -1.0, 0.2) == pytest.approx(-0.2)
    assert rate_limit(0.0, 0.1, 0.2) == pytest.approx(0.1)
