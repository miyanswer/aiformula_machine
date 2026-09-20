"""
2 周目以降の走行ライン: 1 周目に記録した左右境界の断面ごとにウェイポイントを 1 つ置き,
2 次計画法 (QP) で「曲率 (2 階差分) の二乗和が最小」になる位置を決める = 最小曲率ライン.
これにより自然にアウト・イン・アウトの経路になる.

    P_i(alpha_i) = L_i + alpha_i * (R_i - L_i),   alpha_i in [m_i, 1 - m_i]
    m_i = (車幅/2 + 安全マージン) / 道幅_i

    min_alpha  sum_i || D2 P ||_i^2  +  lambda_c * sum_i (alpha_i - 0.5)^2  +  lambda_s * sum_i (alpha_{i+1}-alpha_i)^2
    s.t.       m_i <= alpha_i <= 1 - m_i

断面の間隔は不均一 (直線は疎, カーブは密) なので, 2 階差分は不等間隔の差分公式
    D2P_i = 2/(h_{i-1}+h_i) * [ (P_{i+1}-P_i)/h_i - (P_i-P_{i-1})/h_{i-1} ]
を使う (h は断面中点どうしの距離). 周回コースなので添字は mod N.

目的関数は alpha の 2 次式・制約は箱型なので, 箱制約付き最小二乗
    min ||A alpha - b||^2  s.t. lo <= alpha <= hi
として FISTA (加速付き射影勾配法) で解く. 依存ライブラリなしで JS 版 (web_simulator) と
同じ結果になるようにしている. h は解いた経路で更新して数回反復する.

速度プロファイル: v_i = min(v_max, sqrt(a_lat / |kappa_i|)) を前後方向の加減速制限で整形.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class RacelineParams:
    vehicle_half_width: float = 0.40
    safety_margin: float = 0.35
    lambda_center: float = 1e-4      # 中央寄せ (直線部で解を一意にするための弱い正則化)
    lambda_smooth: float = 1e-3      # alpha の隣接差 (ウェイポイントの横ずれの滑らかさ)
    outer_iterations: int = 3        # h を更新して解き直す回数
    fista_iterations: int = 3000
    fista_tol: float = 1e-9
    # 速度プロファイル
    v_max: float = 3.0
    v_min: float = 0.8
    a_lat_max: float = 1.5
    a_accel: float = 1.0
    a_decel: float = 1.5


@dataclass
class Raceline:
    points: np.ndarray       # (N, 2) ウェイポイント (odom 座標)
    alpha: np.ndarray        # (N,) 0=左境界, 1=右境界
    left: np.ndarray         # (N, 2)
    right: np.ndarray        # (N, 2)
    kappa: np.ndarray        # (N,) 符号付き曲率
    speed: np.ndarray        # (N,) 目標速度
    s: np.ndarray            # (N,) 周回累積距離
    length: float


def _second_diff_matrix(h_prev: np.ndarray, h_next: np.ndarray) -> np.ndarray:
    """閉ループ・不等間隔の 2 階差分行列 D (N x N)."""
    n = len(h_prev)
    D = np.zeros((n, n))
    for i in range(n):
        a = 2.0 / (h_prev[i] + h_next[i])
        D[i, (i - 1) % n] += a / h_prev[i]
        D[i, i] -= a * (1.0 / h_prev[i] + 1.0 / h_next[i])
        D[i, (i + 1) % n] += a / h_next[i]
    return D


def _spacing(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    nxt = np.roll(points, -1, axis=0)
    h_next = np.maximum(np.linalg.norm(nxt - points, axis=1), 1e-3)
    h_prev = np.roll(h_next, 1)
    return h_prev, h_next


def solve_box_lsq(A: np.ndarray, b: np.ndarray, lo: np.ndarray, hi: np.ndarray,
                  x0: np.ndarray, iters: int = 3000, tol: float = 1e-9) -> np.ndarray:
    """min 0.5||Ax-b||^2 s.t. lo<=x<=hi を FISTA で解く."""
    Q = A.T @ A
    c = A.T @ b
    # リプシッツ定数 = Q の最大固有値 (べき乗法)
    v = np.ones(Q.shape[0]) / math.sqrt(Q.shape[0])
    for _ in range(100):
        w = Q @ v
        nrm = np.linalg.norm(w)
        if nrm < 1e-15:
            break
        v = w / nrm
    L = max(float(v @ Q @ v), 1e-12) * 1.01
    x = np.clip(x0, lo, hi)
    y = x.copy()
    t = 1.0
    for _ in range(iters):
        x_new = np.clip(y - (Q @ y - c) / L, lo, hi)
        t_new = 0.5 * (1.0 + math.sqrt(1.0 + 4.0 * t * t))
        y = x_new + ((t - 1.0) / t_new) * (x_new - x)
        if np.max(np.abs(x_new - x)) < tol:
            x = x_new
            break
        x, t = x_new, t_new
    return x


def signed_curvature(points: np.ndarray) -> np.ndarray:
    """閉ループ点列の 3 点外接円による符号付き曲率 (左旋回が正)."""
    p0 = np.roll(points, 1, axis=0)
    p2 = np.roll(points, -1, axis=0)
    a = np.linalg.norm(points - p0, axis=1)
    b = np.linalg.norm(p2 - points, axis=1)
    c = np.linalg.norm(p2 - p0, axis=1)
    cross = (points[:, 0] - p0[:, 0]) * (p2[:, 1] - p0[:, 1]) - (points[:, 1] - p0[:, 1]) * (p2[:, 0] - p0[:, 0])
    return 2.0 * cross / np.maximum(a * b * c, 1e-9)


def speed_profile(points: np.ndarray, kappa: np.ndarray, p: RacelineParams) -> np.ndarray:
    n = len(points)
    _, h_next = _spacing(points)
    v = np.minimum(p.v_max, np.sqrt(p.a_lat_max / np.maximum(np.abs(kappa), 1e-6)))
    v = np.maximum(v, p.v_min)
    # 閉ループなので 2 周分まわして収束させる
    for _ in range(2):
        for i in range(n):              # 加速制限 (前向き)
            j = (i + 1) % n
            v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2.0 * p.a_accel * h_next[i]))
        for i in range(n - 1, -1, -1):  # 減速制限 (後ろ向き)
            j = (i + 1) % n
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2.0 * p.a_decel * h_next[i]))
    return np.maximum(v, p.v_min)


def optimize_raceline(left: np.ndarray, right: np.ndarray, p: RacelineParams = RacelineParams(),
                      alpha0: Optional[np.ndarray] = None) -> Raceline:
    left = np.asarray(left, dtype=np.float64)
    right = np.asarray(right, dtype=np.float64)
    n = len(left)
    if n < 5:
        raise ValueError(f"断面が少なすぎます ({n} 点). 1 周目の記録を確認してください")
    span = right - left
    width = np.maximum(np.linalg.norm(span, axis=1), 1e-3)
    margin = np.minimum((p.vehicle_half_width + p.safety_margin) / width, 0.5)
    lo, hi = margin, 1.0 - margin

    alpha = np.full(n, 0.5) if alpha0 is None else np.clip(alpha0, lo, hi)
    for _ in range(p.outer_iterations):
        pts = left + alpha[:, None] * span
        h_prev, h_next = _spacing(pts)
        D = _second_diff_matrix(h_prev, h_next)
        # D2P = D @ (L + diag(alpha) S) -> x, y 成分それぞれ alpha に線形
        Ax = D * span[:, 0][None, :]
        Ay = D * span[:, 1][None, :]
        bx = -(D @ left[:, 0])
        by = -(D @ left[:, 1])
        # 正則化項: 中央寄せ + 隣接差
        Dl = np.eye(n) - np.roll(np.eye(n), 1, axis=1)
        A = np.vstack([Ax, Ay, math.sqrt(p.lambda_center) * np.eye(n), math.sqrt(p.lambda_smooth) * Dl])
        b = np.concatenate([bx, by, math.sqrt(p.lambda_center) * np.full(n, 0.5), np.zeros(n)])
        alpha = solve_box_lsq(A, b, lo, hi, alpha, p.fista_iterations, p.fista_tol)

    pts = left + alpha[:, None] * span
    kappa = signed_curvature(pts)
    speed = speed_profile(pts, kappa, p)
    _, h_next = _spacing(pts)
    s = np.concatenate([[0.0], np.cumsum(h_next[:-1])])
    return Raceline(pts, alpha, left, right, kappa, speed, s, float(np.sum(h_next)))
