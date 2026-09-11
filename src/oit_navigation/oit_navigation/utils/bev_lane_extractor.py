"""
Direct 2D BEV Lane Line Extraction with Quality-Anchored Single-Side Offset Tracking.
"""

from typing import Optional, Tuple, List
import cv2
import numpy as np


class BEVLaneExtractor:
    """
    High-Reliability 2D BEV Lane Line Extractor:
    1. Nearest-to-Vehicle Anchor Search: Always locks onto closest inner lane.
    2. Inertial Sliding Window Tracing: Drifts through dashed gaps to maintain continuous tracking.
    3. Quality Confidence Scoring: Measures line length, density, and polynomial residual.
    4. Anchor-First 3.5m Road Offset: If one lane is cleaner than the other (e.g., solid vs dashed/noisy),
       it prioritizes the clean lane and shifts +-1.75m to produce an ultra-smooth, stable trajectory.
    5. Curvature-Regularized Smoothing: Prevents noisy high-order oscillations.
    """

    def __init__(
        self,
        bev_w: int = 640,
        bev_h: int = 640,
        lane_width_m: float = 3.5,
        dist_min: float = 1.15,
        dist_max: float = 10.0,
        lateral_max: float = 5.0,
        n_windows: int = 14,
        margin_px: int = 45,
        min_recenter_pixels: int = 15,
        min_pixels: int = 15,
        ema_alpha: float = 0.60,
    ):
        self.bev_w = bev_w
        self.bev_h = bev_h
        self.lane_width_m = lane_width_m
        self.half_width_m = lane_width_m / 2.0  # 1.75m
        self.dist_min = dist_min
        self.dist_max = dist_max
        self.lateral_max = lateral_max
        self.n_windows = n_windows
        self.margin_px = margin_px
        self.min_recenter_pixels = min_recenter_pixels
        self.min_pixels = min_pixels
        self.ema_alpha = ema_alpha

        self.v_near = (1.0 - (self.dist_min / self.dist_max)) * self.bev_h

        # Temporal EMA filter states
        self.smoothed_left_coeffs: Optional[np.ndarray] = None
        self.smoothed_right_coeffs: Optional[np.ndarray] = None
        self.smoothed_center_coeffs: Optional[np.ndarray] = None

    def extract_lane_trajectories(
        self,
        bev_mask: np.ndarray,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], np.ndarray]:
        """
        Extracts left and right lines, evaluates line quality, and derives a rock-solid
        target racing line by anchoring onto the cleanest continuous lane boundary (+-1.75m offset).
        """
        annotated_bev = cv2.cvtColor((bev_mask > 0).astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)

        nonzero = bev_mask.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        sample_x = np.linspace(0.0, 7.5, num=50)

        if len(nonzerox) < 20:
            target_center_metric = self._extrapolate_previous_center(sample_x)
            return target_center_metric, None, None, annotated_bev

        # 1. Base Anchor Peak Search (Nearest-to-Vehicle Anchor)
        mid_u = self.bev_w // 2
        bottom_start = int(self.v_near * 0.40)
        bottom_end = int(self.v_near)

        hist_roi = bev_mask[bottom_start:bottom_end, :]
        histogram = np.sum(hist_roi, axis=0).astype(float)
        histogram = cv2.GaussianBlur(histogram.reshape(1, -1), (15, 1), 3.0).flatten()

        left_base = self._find_nearest_peak(histogram[:mid_u], is_left=True, mid_u=mid_u)
        right_base = self._find_nearest_peak(histogram[mid_u:], is_left=False, mid_u=mid_u)

        # 2. Inertial Sliding Window Tracing
        window_height = int(self.v_near / self.n_windows)
        left_current = left_base
        right_current = right_base

        left_dx = 0.0
        right_dx = 0.0

        left_lane_inds = []
        right_lane_inds = []

        for window in range(self.n_windows):
            win_y_low = int(self.v_near - (window + 1) * window_height)
            win_y_high = int(self.v_near - window * window_height)

            win_xleft_low = int(left_current - self.margin_px)
            win_xleft_high = int(left_current + self.margin_px)
            win_xright_low = int(right_current - self.margin_px)
            win_xright_high = int(right_current + self.margin_px)

            # Draw search boxes onto debug image
            cv2.rectangle(annotated_bev, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (100, 60, 0), 1)
            cv2.rectangle(annotated_bev, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 80, 110), 1)

            good_left_inds = (
                (nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)
            ).nonzero()[0]
            good_right_inds = (
                (nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)
            ).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # Left window recentering & inertial drift
            if len(good_left_inds) >= self.min_recenter_pixels:
                new_left = np.mean(nonzerox[good_left_inds])
                left_dx = 0.7 * left_dx + 0.3 * (new_left - left_current)
                left_current = new_left
            else:
                left_current += left_dx

            # Right window recentering & inertial drift
            if len(good_right_inds) >= self.min_recenter_pixels:
                new_right = np.mean(nonzerox[good_right_inds])
                right_dx = 0.7 * right_dx + 0.3 * (new_right - right_current)
                right_current = new_right
            else:
                right_current += right_dx

        left_lane_inds = np.concatenate(left_lane_inds) if len(left_lane_inds) > 0 else np.array([], dtype=int)
        right_lane_inds = np.concatenate(right_lane_inds) if len(right_lane_inds) > 0 else np.array([], dtype=int)

        left_pts_metric: Optional[np.ndarray] = None
        right_pts_metric: Optional[np.ndarray] = None

        if len(left_lane_inds) >= 15:
            lx, ly = nonzerox[left_lane_inds], nonzeroy[left_lane_inds]
            annotated_bev[ly, lx] = [255, 180, 0]  # Cyan
            left_pts_metric = self._px_to_metric(lx, ly)

        if len(right_lane_inds) >= 15:
            rx, ry = nonzerox[right_lane_inds], nonzeroy[right_lane_inds]
            annotated_bev[ry, rx] = [0, 220, 255]  # Yellow
            right_pts_metric = self._px_to_metric(rx, ry)

        # 3. Fit Polynomials & Compute Quality / Confidence Scores
        l_coeffs, l_qual = self._fit_and_score(left_pts_metric)
        r_coeffs, r_qual = self._fit_and_score(right_pts_metric)

        # Temporal EMA smoothing on polynomial coefficients
        if l_coeffs is not None:
            if self.smoothed_left_coeffs is None:
                self.smoothed_left_coeffs = l_coeffs
            else:
                self.smoothed_left_coeffs = self.ema_alpha * l_coeffs + (1.0 - self.ema_alpha) * self.smoothed_left_coeffs

        if r_coeffs is not None:
            if self.smoothed_right_coeffs is None:
                self.smoothed_right_coeffs = r_coeffs
            else:
                self.smoothed_right_coeffs = self.ema_alpha * r_coeffs + (1.0 - self.ema_alpha) * self.smoothed_right_coeffs

        # 4. Anchor-First 3.5m Road Offset Strategy
        target_center_metric: Optional[np.ndarray] = None

        has_left = (self.smoothed_left_coeffs is not None and l_qual > 0.15)
        has_right = (self.smoothed_right_coeffs is not None and r_qual > 0.15)

        if has_left and has_right:
            if l_qual >= 1.4 * r_qual:
                # Left lane is much cleaner -> Shift right by half lane width (+1.75m)
                l_y = np.polyval(self.smoothed_left_coeffs, sample_x)
                mid_y = l_y - self.half_width_m
                target_center_metric = np.column_stack((sample_x, mid_y))
            elif r_qual >= 1.4 * l_qual:
                # Right lane is much cleaner -> Shift left by half lane width (-1.75m)
                r_y = np.polyval(self.smoothed_right_coeffs, sample_x)
                mid_y = r_y + self.half_width_m
                target_center_metric = np.column_stack((sample_x, mid_y))
            else:
                # Both lanes are high quality -> Weighted fusion from both sides
                l_y = np.polyval(self.smoothed_left_coeffs, sample_x) - self.half_width_m
                r_y = np.polyval(self.smoothed_right_coeffs, sample_x) + self.half_width_m
                w_tot = l_qual + r_qual
                mid_y = (l_qual * l_y + r_qual * r_y) / w_tot
                target_center_metric = np.column_stack((sample_x, mid_y))

        elif has_left:
            # Only Left lane is reliable -> Shift right by +1.75m
            l_y = np.polyval(self.smoothed_left_coeffs, sample_x)
            mid_y = l_y - self.half_width_m
            target_center_metric = np.column_stack((sample_x, mid_y))

        elif has_right:
            # Only Right lane is reliable -> Shift left by -1.75m
            r_y = np.polyval(self.smoothed_right_coeffs, sample_x)
            mid_y = r_y + self.half_width_m
            target_center_metric = np.column_stack((sample_x, mid_y))

        else:
            target_center_metric = self._extrapolate_previous_center(sample_x)

        # Smooth and store center model
        if target_center_metric is not None:
            c_fit = self._fit_regularized_poly(target_center_metric)
            if c_fit is not None:
                if self.smoothed_center_coeffs is None:
                    self.smoothed_center_coeffs = c_fit
                else:
                    self.smoothed_center_coeffs = self.ema_alpha * c_fit + (1.0 - self.ema_alpha) * self.smoothed_center_coeffs
                mid_y_smooth = np.polyval(self.smoothed_center_coeffs, sample_x)
                target_center_metric = np.column_stack((sample_x, mid_y_smooth))

        # Draw Target Path onto BEV image (Thick Bright Green line)
        if target_center_metric is not None and len(target_center_metric) >= 2:
            path_px = [self._metric_to_px(float(pt[0]), float(pt[1])) for pt in target_center_metric]
            for i in range(len(path_px) - 1):
                p1, p2 = path_px[i], path_px[i + 1]
                if 0 <= p1[0] < self.bev_w and 0 <= p1[1] < self.bev_h and \
                   0 <= p2[0] < self.bev_w and 0 <= p2[1] < self.bev_h:
                    cv2.line(annotated_bev, p1, p2, (0, 255, 0), 4)

            # Draw 5.0m lookahead target anchor on BEV
            if len(path_px) > 25:
                far_px = path_px[min(len(path_px)-1, 33)]
                if 0 <= far_px[0] < self.bev_w and 0 <= far_px[1] < self.bev_h:
                    cv2.circle(annotated_bev, far_px, 8, (0, 255, 50), -1)

        # Draw Vehicle Position (Center bottom anchor)
        ego_u = self.bev_w // 2
        ego_v = int(self.v_near)
        cv2.circle(annotated_bev, (ego_u, ego_v), 9, (255, 100, 0), -1)
        cv2.putText(annotated_bev, "EGO", (ego_u - 15, ego_v + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return target_center_metric, left_pts_metric, right_pts_metric, annotated_bev

    def _find_nearest_peak(self, sub_hist: np.ndarray, is_left: bool, mid_u: int) -> int:
        """Finds the peak closest to vehicle center (rejects outer double lines)."""
        max_val = np.max(sub_hist) if len(sub_hist) > 0 else 0
        if max_val < 10:
            return (mid_u - int(self.bev_w * 0.22)) if is_left else (mid_u + int(self.bev_w * 0.22))

        threshold = max_val * 0.35
        peaks = []
        for i in range(1, len(sub_hist) - 1):
            if sub_hist[i] > threshold and sub_hist[i] >= sub_hist[i - 1] and sub_hist[i] >= sub_hist[i + 1]:
                peaks.append(i)

        if not peaks:
            return (int(np.argmax(sub_hist))) if is_left else (int(np.argmax(sub_hist) + mid_u))

        if is_left:
            return peaks[-1]
        else:
            return mid_u + peaks[0]

    def _fit_and_score(self, pts_metric: Optional[np.ndarray]) -> Tuple[Optional[np.ndarray], float]:
        """Fits curve and computes Quality Score Q in [0.0, 1.0]."""
        if pts_metric is None or len(pts_metric) < 6:
            return None, 0.0

        coeffs = self._fit_regularized_poly(pts_metric)
        if coeffs is None:
            return None, 0.0

        x = pts_metric[:, 0]
        y = pts_metric[:, 1]
        span = float(np.max(x) - np.min(x))
        span_score = np.clip(span / 5.0, 0.0, 1.0)
        density_score = np.clip(len(x) / 60.0, 0.0, 1.0)

        y_eval = np.polyval(coeffs, x)
        mse = float(np.mean((y - y_eval)**2))
        smooth_score = np.clip(1.0 / (1.0 + 10.0 * mse), 0.0, 1.0)

        quality = 0.50 * span_score + 0.30 * density_score + 0.20 * smooth_score
        return coeffs, quality

    def _fit_regularized_poly(self, pts_metric: np.ndarray) -> Optional[np.ndarray]:
        """Curvature-regularized 2nd-order polynomial fit."""
        try:
            if len(pts_metric) < 4:
                return None

            x = pts_metric[:, 0]
            y = pts_metric[:, 1]

            if (np.max(x) - np.min(x)) < 1.0 or len(x) < 8:
                b, c = np.polyfit(x, y, deg=1)
                return np.array([0.0, b, c])

            X_mat = np.column_stack((x**2, x, np.ones_like(x)))
            reg_lambda = 0.30
            reg_matrix = np.diag([reg_lambda, 0.01, 0.001])

            coeffs = np.linalg.inv(X_mat.T @ X_mat + reg_matrix) @ (X_mat.T @ y)
            coeffs[0] = np.clip(coeffs[0], -0.10, 0.10)
            return coeffs
        except Exception:
            return None

    def _extrapolate_previous_center(self, sample_x: np.ndarray) -> Optional[np.ndarray]:
        if self.smoothed_center_coeffs is not None:
            mid_y = np.polyval(self.smoothed_center_coeffs, sample_x)
            return np.column_stack((sample_x, mid_y))
        return None

    def _px_to_metric(self, u_arr: np.ndarray, v_arr: np.ndarray) -> np.ndarray:
        xs = self.dist_max - (v_arr / self.v_near) * (self.dist_max - self.dist_min)
        ys = self.lateral_max - (u_arr / self.bev_w) * (2.0 * self.lateral_max)
        valid = (xs >= self.dist_min) & (xs <= self.dist_max)
        if not np.any(valid):
            return np.empty((0, 2))
        pts = np.column_stack((xs[valid], ys[valid]))
        return pts[np.argsort(pts[:, 0])]

    def _metric_to_px(self, x: float, y: float) -> Tuple[int, int]:
        u = int(np.round((self.lateral_max - y) / (2.0 * self.lateral_max) * self.bev_w))
        v = int(np.round((self.dist_max - x) / (self.dist_max - self.dist_min) * self.v_near))
        return u, v
