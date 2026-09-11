"""
Inverse Perspective Mapping (IPM) for Bird's-Eye View (BEV) Transformation.
"""

from typing import Tuple, Optional
import cv2
import numpy as np


class BEVTransformer:
    """
    Transforms camera perspective image/mask into metric Bird's-Eye View (BEV) space
    and provides bidirectional coordinate conversions between pixel space and robot frame.
    """

    def __init__(
        self,
        src_w: int = 1920,
        src_h: int = 1080,
        cam_height: float = 0.56,       # Camera ground clearance [m]
        cam_focal: float = 1050.0,      # 1080p equivalent focal length [px]
        dist_min: float = 1.15,         # Forward distance min [m]
        dist_max: float = 10.0,         # Forward distance max [m]
        lateral_max: float = 5.0,       # Lateral half-width (+-5.0m = 10.0m total) [m]
        bev_w: int = 640,               # Output BEV image width [px]
        bev_h: int = 640,               # Output BEV image height [px]
    ):
        self.src_w = src_w
        self.src_h = src_h
        self.cam_height = cam_height
        self.cam_focal = cam_focal
        self.dist_min = dist_min
        self.dist_max = dist_max
        self.lateral_max = lateral_max
        self.bev_w = bev_w
        self.bev_h = bev_h

        # Compute Homography Matrix
        self.M, self.M_inv = self._build_transform_matrix()

        # Meters per pixel scales
        self.v_near = (1.0 - (self.dist_min / self.dist_max)) * self.bev_h
        self.scale_x = (self.dist_max - self.dist_min) / self.v_near  # [m/px] along longitudinal axis
        self.scale_y = (2.0 * self.lateral_max) / self.bev_w          # [m/px] along lateral axis

    def _build_transform_matrix(self) -> Tuple[np.ndarray, np.ndarray]:
        cx = self.src_w / 2.0
        cy = self.src_h / 2.0
        f_px = self.cam_focal * (self.src_w / 1920.0)

        # 4 corners in real-world ground projected to image coordinates (u, v)
        src_pts = np.float32([
            [cx - f_px * (self.lateral_max / self.dist_min), cy + f_px * (self.cam_height / self.dist_min)],  # Near-Left
            [cx + f_px * (self.lateral_max / self.dist_min), cy + f_px * (self.cam_height / self.dist_min)],  # Near-Right
            [cx + f_px * (self.lateral_max / self.dist_max), cy + f_px * (self.cam_height / self.dist_max)],  # Far-Right
            [cx - f_px * (self.lateral_max / self.dist_max), cy + f_px * (self.cam_height / self.dist_max)],  # Far-Left
        ])

        v_near = (1.0 - (self.dist_min / self.dist_max)) * self.bev_h
        dst_pts = np.float32([
            [0, v_near],
            [self.bev_w, v_near],
            [self.bev_w, 0],
            [0, 0],
        ])

        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        M_inv = cv2.getPerspectiveTransform(dst_pts, src_pts)
        return M, M_inv

    def warp_to_bev(self, image_or_mask: np.ndarray, is_binary: bool = False) -> np.ndarray:
        """Warps camera image or mask into 2D metric BEV, auto-adapting to image shape."""
        h, w = image_or_mask.shape[:2]
        if w != self.src_w or h != self.src_h:
            self.src_w = w
            self.src_h = h
            self.M, self.M_inv = self._build_transform_matrix()

        flags = cv2.INTER_NEAREST if is_binary else cv2.INTER_LINEAR
        bev = cv2.warpPerspective(image_or_mask, self.M, (self.bev_w, self.bev_h), flags=flags)
        return bev

    def bev_px_to_robot_xy(self, u: float, v: float) -> Tuple[float, float]:
        """
        Converts BEV pixel coordinate (u, v) to robot metric coordinates (x, y) in base_link.
        x: forward distance [m] (positive forward)
        y: lateral distance [m] (positive left, negative right)
        """
        x = self.dist_max - (v / self.v_near) * (self.dist_max - self.dist_min)
        y = self.lateral_max - (u / self.bev_w) * (2.0 * self.lateral_max)
        return float(x), float(y)

    def robot_xy_to_bev_px(self, x: float, y: float) -> Tuple[int, int]:
        """Converts robot metric coordinates (x, y) to BEV pixel coordinates (u, v)."""
        u = int(np.round((self.lateral_max - y) / (2.0 * self.lateral_max) * self.bev_w))
        v = int(np.round((self.dist_max - x) / (self.dist_max - self.dist_min) * self.v_near))
        return u, v

    def extract_bev_points_metric(self, bev_mask: np.ndarray, min_x: float = 1.0, max_x: float = 8.5) -> np.ndarray:
        """
        Extracts all positive pixels from BEV binary mask and converts directly to metric (x, y) points.
        Returns:
            np.ndarray of shape (N, 2) containing metric (x, y) points in base_link frame.
        """
        v_coords, u_coords = np.nonzero(bev_mask > 0)
        if len(u_coords) == 0:
            return np.empty((0, 2), dtype=np.float64)

        xs = self.dist_max - (v_coords / self.v_near) * (self.dist_max - self.dist_min)
        ys = self.lateral_max - (u_coords / self.bev_w) * (2.0 * self.lateral_max)

        valid_mask = (xs >= min_x) & (xs <= max_x)
        xs_val = xs[valid_mask]
        ys_val = ys[valid_mask]

        if len(xs_val) == 0:
            return np.empty((0, 2), dtype=np.float64)

        return np.column_stack((xs_val, ys_val))
