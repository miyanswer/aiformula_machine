"""motor_controller の ROS 非依存な計算 (単体テスト: test/test_motor_math.py).

    速度指令 (v, omega) -> 左右車輪 RPM -> CAN フレーム 0x210 のデータ, 加減速制限.
    odometry_publisher/wheel.hpp (RPM -> 速度) と逆変換の関係.
"""

import math
from typing import List, Tuple


def rate_limit(current: float, target: float, max_step: float) -> float:
    return current + max(-max_step, min(max_step, target - current))


def rate_limit_linear(current: float, target: float, dt: float, max_accel: float, max_decel: float) -> float:
    """|v| が小さくなる向き (減速・停止・前後反転の手前まで) は max_decel, 大きくなる向きは max_accel で制限する."""
    if current * target < 0.0:
        return rate_limit(current, 0.0, max_decel * dt)
    if abs(target) < abs(current):
        return rate_limit(current, target, max_decel * dt)
    return rate_limit(current, target, max_accel * dt)


def to_ref_rpm(linear_velocity: float, angular_velocity: float, diameter: float, tread: float,
               gear_ratio: float) -> Tuple[float, float]:
    """(左 RPM, 右 RPM). 左右の符号が逆になる (その場旋回) 指令は両輪 0 にする."""
    k = linear_velocity / (diameter * 0.5)
    left = k - (tread / diameter) * angular_velocity   # [rad/s]
    right = k + (tread / diameter) * angular_velocity
    to_rpm = 60.0 / (2.0 * math.pi)
    left, right = left * to_rpm, right * to_rpm
    if left * right < 0.0:
        left = right = 0.0
    return left * gear_ratio, right * gear_ratio


def to_can_cmd(rpm: float) -> List[int]:
    return list(round(rpm).to_bytes(4, 'little', signed=True))


def to_can_data(linear_velocity: float, angular_velocity: float, diameter: float, tread: float,
                gear_ratio: float) -> List[int]:
    """CAN 0x210 のデータ 8 byte: 右 RPM (int32 LE) + 左 RPM (int32 LE)."""
    left, right = to_ref_rpm(linear_velocity, angular_velocity, diameter, tread, gear_ratio)
    return to_can_cmd(right) + to_can_cmd(left)
