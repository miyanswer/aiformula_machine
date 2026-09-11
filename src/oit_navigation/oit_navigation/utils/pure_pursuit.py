"""
Adaptive Pure Pursuit and Geometric Steering Controller.
"""

import math
from typing import Optional, Tuple
import numpy as np


class AdaptivePurePursuit:
    """
    Adaptive Pure Pursuit controller with speed-dependent lookahead distance
    and polynomial trajectory tracking.
    """

    def __init__(
        self,
        min_lookahead: float = 1.2,
        max_lookahead: float = 6.5,
        lookahead_ratio: float = 0.5,
        angular_gain: float = 1.0,
        far_weight_gain: float = 1.3,
        max_angular_vel: float = 1.6,
    ):
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.lookahead_ratio = lookahead_ratio
        self.angular_gain = angular_gain
        self.far_weight_gain = far_weight_gain
        self.max_angular_vel = max_angular_vel

    def compute_lookahead_distance(self, current_speed: float) -> float:
        """Calculates dynamic lookahead distance proportional to vehicle speed."""
        l_dist = self.min_lookahead + self.lookahead_ratio * max(0.0, current_speed)
        return float(np.clip(l_dist, self.min_lookahead, self.max_lookahead))

    def find_target_point_on_curve(
        self,
        poly_coeffs: np.ndarray,
        lookahead_distance: float,
    ) -> Tuple[float, float]:
        """
        Finds the exact intersection point (x_t, y_t) on the fitted curve y = poly(x)
        at forward lookahead distance.
        """
        x_target = lookahead_distance
        y_target = float(np.polyval(poly_coeffs, x_target))
        return x_target, y_target

    def compute_steering(
        self,
        target_point: Tuple[float, float],
        current_speed: float,
    ) -> Tuple[float, float]:
        """
        Calculates angular velocity command (omega) and instantaneous road curvature (kappa).
        Returns:
            (angular_velocity [rad/s], curvature [1/m])
        """
        tx, ty = target_point
        d2 = tx**2 + ty**2
        if d2 < 1e-4:
            return 0.0, 0.0

        # Curvature of the circle connecting origin to target point: kappa = 2*y / d^2
        kappa = 2.0 * ty / d2

        # Pure Pursuit angular velocity: omega = 2 * v * sin(alpha) / d
        alpha = math.atan2(ty, tx)
        effective_speed = max(current_speed, 0.35)
        omega = self.angular_gain * (2.0 * math.sin(alpha) / math.sqrt(d2)) * effective_speed

        omega_clamped = float(np.clip(omega, -self.max_angular_vel, self.max_angular_vel))
        return omega_clamped, kappa
