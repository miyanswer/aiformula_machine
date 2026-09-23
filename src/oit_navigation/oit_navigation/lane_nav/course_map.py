"""
1 周目の記録 (BoundarySample 列) からコースマップ (左右境界の断面列) を作る.

    - 周回検出: 走行距離が min_lap_length を超えた後, 最初の断面の近く (close_radius 以内,
      方位差 close_heading 以内) に戻ってきたら 1 周完了.
    - ループ閉じ込み: 最後の断面の先に最初の断面が来るはずなので, 最後の断面から見た
      最初の断面の「横ずれ」(進行方向の法線成分) をオドメトリの蓄積誤差とみなし,
      走行距離に比例させて全断面に配分して打ち消す (位置のみ. 方位ドリフトは小さい前提).
    - 方位ドリフト補正: ジャイロのバイアスで odom の方位は時間とともにずれる. スタート時と 1 周後の
      「白線 (中央線) の絶対方位 = 車両方位 + 車両から見た白線の向き」の差 - 2pi がそのずれなので,
      走行距離に比例して方位がずれたとみなして記録時の姿勢を組み直し, 左右境界点を再計算する.
    - JSON で保存/読み込みできる (2 周目からの再起動用).
"""

import json
import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from .boundary_recorder import BoundarySample, vehicle_to_world


@dataclass
class LapDetectorParams:
    min_lap_length: float = 30.0
    close_radius: float = 2.5
    close_heading: float = math.radians(45.0)
    loop_closure: bool = True
    max_closure_correction: float = 1.5   # これ以上の横ずれは誤検出とみなし補正しない [m]
    # 方位ドリフト補正: 白線の向きの推定ノイズ (シミュレータで 0.04-0.1 rad) が実際のドリフト
    # (停止中にジャイロバイアスを推定すれば 1 周で数十 mrad 以下) より大きくなりやすいので既定は無効.
    # 推定値自体は lane_navigator の status (yaw_drift) に出る.
    yaw_drift_correction: bool = False
    max_yaw_drift_correction: float = math.radians(20.0)


@dataclass
class CourseMap:
    left: np.ndarray          # (N, 2)
    right: np.ndarray         # (N, 2)
    center: np.ndarray        # (N, 2) 記録時の車両経路上 (x_rec 前方) の点
    s: np.ndarray             # (N,) 記録時走行距離
    left_detected: np.ndarray
    right_detected: np.ndarray
    start_pose: tuple         # 1 周目開始時の (x, y, yaw)
    closure_error: float = 0.0
    # 各断面に掛けたループ閉じ込みの補正量 (N, 2). 1 周目に記憶したコーンを境界と同じ座標に載せるのに使う
    closure_offsets: Optional[np.ndarray] = None

    def to_json(self) -> str:
        return json.dumps({
            "left": self.left.tolist(), "right": self.right.tolist(),
            "center": self.center.tolist(), "s": self.s.tolist(),
            "left_detected": self.left_detected.astype(int).tolist(),
            "right_detected": self.right_detected.astype(int).tolist(),
            "start_pose": list(self.start_pose), "closure_error": self.closure_error,
            **({"closure_offsets": self.closure_offsets.tolist()} if self.closure_offsets is not None else {}),
        })

    @staticmethod
    def from_json(text: str) -> "CourseMap":
        d = json.loads(text)
        return CourseMap(
            np.asarray(d["left"], float), np.asarray(d["right"], float),
            np.asarray(d["center"], float), np.asarray(d["s"], float),
            np.asarray(d["left_detected"], bool), np.asarray(d["right_detected"], bool),
            tuple(d["start_pose"]), float(d.get("closure_error", 0.0)),
            np.asarray(d["closure_offsets"], float) if "closure_offsets" in d else None,
        )


def lap_completed(samples: List[BoundarySample], pose, s_now: float, start_pose, start_s: float,
                  p: LapDetectorParams) -> bool:
    if len(samples) < 5 or s_now - start_s < p.min_lap_length:
        return False
    dx = pose[0] - start_pose[0]
    dy = pose[1] - start_pose[1]
    dyaw = abs(math.atan2(math.sin(pose[2] - start_pose[2]), math.cos(pose[2] - start_pose[2])))
    return math.hypot(dx, dy) <= p.close_radius and dyaw <= p.close_heading


def yaw_drift_applied(p: LapDetectorParams, yaw_drift: float) -> bool:
    """build_course_map が方位ドリフト補正を掛けるか (コーン記憶も同じ条件で合わせる)."""
    return p.yaw_drift_correction and yaw_drift != 0.0 and abs(yaw_drift) <= p.max_yaw_drift_correction


def corrected_poses(samples: List[BoundarySample], yaw_drift: float) -> List[np.ndarray]:
    """方位ドリフト yaw_drift [rad] (1 周分) を走行距離比例で取り除いた記録時の姿勢列.
    web_simulator/js/lane_navigator.js の correctedPoseSequence() と同じ計算."""
    s0, s1 = samples[0].s, samples[-1].s
    poses = [np.array(samples[0].pose, float)]
    for a, b in zip(samples[:-1], samples[1:]):
        w_mid = ((a.s + b.s) * 0.5 - s0) / max(s1 - s0, 1e-9)
        rot = -yaw_drift * w_mid
        d = np.array(b.pose[:2]) - np.array(a.pose[:2])
        c, s = math.cos(rot), math.sin(rot)
        prev = poses[-1]
        yaw = b.pose[2] - yaw_drift * (b.s - s0) / max(s1 - s0, 1e-9)
        poses.append(np.array([prev[0] + c * d[0] - s * d[1], prev[1] + s * d[0] + c * d[1], yaw]))
    return poses


def correct_yaw_drift(samples: List[BoundarySample], yaw_drift: float, x_rec: float):
    """方位ドリフト yaw_drift [rad] (1 周分) を走行距離比例で取り除いた左右境界点を返す."""
    poses = corrected_poses(samples, yaw_drift)
    left = np.array([vehicle_to_world(ps, x_rec, sm.y_left) for ps, sm in zip(poses, samples)])
    right = np.array([vehicle_to_world(ps, x_rec, sm.y_right) for ps, sm in zip(poses, samples)])
    return left, right


def build_course_map(samples: List[BoundarySample], start_pose, p: LapDetectorParams,
                     yaw_drift: float = 0.0, x_rec: float = 2.0) -> CourseMap:
    left = np.array([s.left for s in samples], float)
    right = np.array([s.right for s in samples], float)
    if yaw_drift_applied(p, yaw_drift):
        left, right = correct_yaw_drift(samples, yaw_drift, x_rec)
    center = 0.5 * (left + right)
    ss = np.array([s.s for s in samples], float)

    # 1 周を超えて記録した分 (最初の断面を追い越した断面) を落とす:
    # 最初の断面の進行方向で見て, 最初の断面より手前 (負側) かつ近い最後尾の断面を残す
    t0 = center[1] - center[0]
    t0 = t0 / max(np.linalg.norm(t0), 1e-9)
    keep = len(samples)
    for i in range(len(samples) - 1, len(samples) // 2, -1):
        d = center[i] - center[0]
        if np.linalg.norm(d) < 3.0 and float(d @ t0) > -0.3:
            keep = i  # この断面は最初の断面と重複 (もしくは追い越し)
        else:
            break
    left, right, center, ss = left[:keep], right[:keep], center[:keep], ss[:keep]
    ld = np.array([s.left_detected for s in samples[:keep]], bool)
    rd = np.array([s.right_detected for s in samples[:keep]], bool)

    closure = 0.0
    offsets = np.zeros_like(center)
    if p.loop_closure and len(center) >= 5:
        # 最後の断面 -> 最初の断面 のベクトルを, 最後の区間の進行方向 t とその法線に分解する.
        # 進行方向成分は「最後の記録から 1 周完了までの隙間」なので誤差ではない.
        # 法線成分 (横ずれ) がオドメトリの蓄積誤差 -> これだけを打ち消す.
        t_last = center[-1] - center[-2]
        t_last = t_last / max(np.linalg.norm(t_last), 1e-9)
        d = center[0] - center[-1]
        gap = max(float(d @ t_last), 1e-3)
        err = -(d - (d @ t_last) * t_last)       # 最初の断面を基準に, 後半の断面を動かす量
        closure = float(np.linalg.norm(err))
        if closure <= p.max_closure_correction:
            # 誤差を走行距離に比例配分 (最初の断面は動かさず, 最後の断面ほど大きく補正)
            w = (ss - ss[0]) / max(ss[-1] - ss[0] + gap, 1e-9)
            corr = -w[:, None] * err[None, :]
            left, right, center = left + corr, right + corr, center + corr
            offsets = corr

    return CourseMap(left, right, center, ss, ld, rd, tuple(start_pose), closure, offsets)
