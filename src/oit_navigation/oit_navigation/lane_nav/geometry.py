"""
画像座標 -> 車両座標 (base_link, 地面) への点投影と, 白線点列の多項式フィット.

BEV 画像への射影変換 (warpPerspective) は行わず, UFLD が返す疎な点 (1 本あたり最大 56 点)
だけをピンホールカメラ + 平面地面モデルで直接地面に落とす.

座標系:
    base_link: x 前方 [m], y 左 [m] (右が負)
    カメラ:     光軸が車両前方を向き, pitch_down [rad] だけ下向き, 取付位置 (cam_x, 0, cam_height)
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class CameraModel:
    """カメラ内部/外部パラメータ. fx, fy, cx, cy は ref_width x ref_height 画像での値."""

    fx: float = 763.17
    fy: float = 763.17
    cx: float = 960.0
    cy: float = 540.0
    ref_width: int = 1920
    ref_height: int = 1080
    cam_height: float = 0.56   # 地面からの高さ [m]
    cam_x: float = 0.055       # base_link からの前方オフセット [m]
    pitch_down: float = np.deg2rad(7.3)

    def scaled(self, width: int, height: int) -> Tuple[float, float, float, float]:
        sx = width / self.ref_width
        sy = height / self.ref_height
        return self.fx * sx, self.fy * sy, self.cx * sx, self.cy * sy


def project_to_ground(
    cam: CameraModel, u: np.ndarray, v: np.ndarray, width: int, height: int,
    min_depression: float = np.deg2rad(1.0),
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """画像点 (u, v) を地面 z=0 に投影し (x, y, valid) を返す.

    水平線付近 (俯角 < min_depression) の点は距離誤差が爆発するので valid=False にする.
    """
    fx, fy, cx, cy = cam.scaled(width, height)
    xc = (np.asarray(u, dtype=np.float64) - cx) / fx   # 右が正
    yc = (np.asarray(v, dtype=np.float64) - cy) / fy   # 下が正
    s, c = np.sin(cam.pitch_down), np.cos(cam.pitch_down)
    ray_fwd = c - yc * s
    ray_down = s + yc * c
    valid = ray_down > np.tan(min_depression) * np.maximum(ray_fwd, 1e-6)
    t = np.where(valid, cam.cam_height / np.where(valid, ray_down, 1.0), 0.0)
    x = cam.cam_x + t * ray_fwd
    y = -t * xc
    return x, y, valid


def ground_to_image(cam: CameraModel, x: np.ndarray, y: np.ndarray, width: int, height: int):
    """project_to_ground の逆: 地面点 (base_link) -> 画像座標 (u, v, valid). 可視化用."""
    fx, fy, cx, cy = cam.scaled(width, height)
    s, c = np.sin(cam.pitch_down), np.cos(cam.pitch_down)
    xr = np.asarray(x, dtype=np.float64) - cam.cam_x
    zr = -cam.cam_height
    zc = c * xr - s * zr                      # 光軸方向の奥行き
    y_down = -(s * xr + c * zr)
    valid = zc > 0.3
    zc = np.where(valid, zc, 1.0)
    u = cx + fx * (-np.asarray(y, dtype=np.float64)) / zc
    v = cy + fy * y_down / zc
    return u, v, valid


@dataclass
class LineFit:
    """白線の車両座標系での 2 次多項式 y = c0 + c1 x + c2 x^2 (x_min..x_max で有効)."""

    coeffs: np.ndarray            # [c0, c1, c2]
    x_min: float
    x_max: float
    n_points: int
    residual: float = 0.0
    inferred: bool = False        # 他の線からの補完で作ったものなら True

    def y_at(self, x):
        c0, c1, c2 = self.coeffs
        return c0 + c1 * x + c2 * x * x

    def heading_at(self, x: float) -> float:
        return float(np.arctan(self.coeffs[1] + 2.0 * self.coeffs[2] * x))

    def curvature_at(self, x: float) -> float:
        d1 = self.coeffs[1] + 2.0 * self.coeffs[2] * x
        return float(2.0 * self.coeffs[2] / (1.0 + d1 * d1) ** 1.5)

    def shifted(self, dy: float, inferred: bool = True) -> "LineFit":
        """横方向に dy [m] 平行移動した線 (曲率が小さい範囲では法線方向オフセットの近似)."""
        c = self.coeffs.copy()
        c[0] += dy
        return LineFit(c, self.x_min, self.x_max, 0, self.residual, inferred)

    def sample(self, step: float = 0.5):
        xs = np.arange(self.x_min, self.x_max + 1e-6, step)
        return xs, self.y_at(xs)


def fit_line(
    x: np.ndarray, y: np.ndarray, x_max_fit: float = 12.0, min_points: int = 3,
    curvature_reg: float = 0.1,
) -> Optional[LineFit]:
    """地面投影した白線点を 2 次多項式でフィットする (遠方点は重みを下げる, 2 次項は正則化).

    点が少ない / 前後方向の広がりが無い場合は None.
    """
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    keep = (x > 0.3) & (x < x_max_fit)
    x, y = x[keep], y[keep]
    if len(x) < min_points or (x.max() - x.min()) < 0.8:
        return None
    # 遠方ほど投影誤差が大きいので 1/x で重み付け
    w = 1.0 / np.maximum(x, 1.0)
    A = np.stack([np.ones_like(x), x, x * x], axis=1) * w[:, None]
    b = y * w
    # 2 次項の正則化 (点数が少ない / 範囲が狭いときに曲がり過ぎないように)
    reg = np.diag([0.0, 0.0, curvature_reg * 10.0])
    coeffs = np.linalg.solve(A.T @ A + reg, A.T @ b)
    res = float(np.sqrt(np.mean((np.stack([np.ones_like(x), x, x * x], 1) @ coeffs - y) ** 2)))
    return LineFit(coeffs, float(x.min()), float(x.max()), int(len(x)), res)
