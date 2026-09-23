"""
UFLD の検出線 (スロット番号付き) を「左境界 / 中央線 / 右境界」の 3 役割に割り当てる.

UFLD のスロット番号は学習データのラベル付け方 (自車レーン基準) に依存し,
「中央白線の上を走る」状況ではスロットと役割が一致しない. そこでスロット番号は使わず,
地面投影した各線の横位置 (x_ref [m] 前方での y) を, 前フレームまでに追跡している
3 本の横位置と照合して割り当てる. 組み合わせは総当たりで,
    - 各役割の予測位置からゲート内 (最近傍)
    - 左 > 中央 > 右 の順序
    - 同時に割り当てる線どうしの間隔が追跡中の道幅と整合 (合流部の分岐線などを弾く)
    - 同時に割り当てる線どうしが概ね平行
を満たすもののうち「割り当て本数が最大 -> 予測とのずれが最小」を選ぶ.
さらに境界線は「中央線の外側で最初に現れる線」とし, 選んだ境界より内側に道幅の整合する線が
あればそちらに付け替える (外側の二重線 = 路肩線を境界と取り違えて道幅が広がるのを防ぐ).

「車両は中央線の上」とは仮定しない (端のレーンを走る 6 レーン走行で右白線を中央線と取り違えるため):
    - 起動時の横位置は init_offset (中央線からの横ずれ, 左正) で指定する
    - 見失い続けたときは最後の横位置を保ったまま, 線の並びと道幅だけ初期値に戻す
    - 3 本 (または左右の境界 2 本) が道幅どおりに揃って見えれば, 割り当てがずれていても
      anchor_frames フレーム続いた時点で付け直す (再アンカー)
    - 2 本しか見えず「左+中央」か「中央+右」か幾何的に区別できないときは二重線を手掛かりにする:
      平行な 2 本が double_line_gap の間隔で並んでいれば境界線 (中央線は 1 本線) で,
      車に近い方が境界, 車から見た側 (左/右) がそのまま役割 (use_double_line で無効化できる)
    - 外部 (6 レーン走行の横位置追跡) から seed_lane_position() で置き直せる

見えなかった線は, 追跡中の道幅 (左-中央, 中央-右 の距離) で隣の線を平行移動して補完する.
"""

import itertools
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .geometry import LineFit

ROLES = ("left", "center", "right")


@dataclass
class LineTrackerParams:
    x_ref: float = 2.0                 # 横位置を比較する前方距離 [m]
    lane_width_init: float = 3.5       # 中央線 <-> 境界線 (左右それぞれ) の距離の初期値 [m]
    lane_width_min: float = 1.0
    lane_width_max: float = 5.0
    width_alpha: float = 0.1           # 道幅 EMA の更新率
    gate_ratio: float = 0.4            # 割り当てゲート = lane_width * gate_ratio
    width_tolerance: float = 0.3       # 線どうしの間隔の許容誤差 (道幅に対する比)
    max_heading_diff: float = 0.3      # 同時に割り当てる線どうしの向きの差の上限 [rad]
    lost_reset_frames: int = 30        # 連続でこのフレーム数何も見えなければ線の並び・道幅を初期値に戻す (横位置は保つ)
    init_offset: float = 0.0           # 起動時の車両位置: 中央線からの横ずれ [m] (左正). 0 = 中央線の上
    anchor_tolerance: float = 0.15     # 再アンカーに使う線どうしの間隔の許容誤差 (道幅に対する比, 通常より厳しく)
    anchor_frames: int = 3             # 再アンカーの根拠がこのフレーム数続いたら付け直す
    anchor_max_x_min: float = 6.0      # 根拠にする線は手前 (この距離以内) から見えているもの (遠方だけの線の外挿は使わない)
    anchor_heading_diff: float = 0.1   # 根拠にする線どうしの向きの差の上限 [rad] (通常より厳しく)
    use_double_line: bool = True       # 二重線 = 境界線 として再アンカーの根拠に使う
    double_line_gap_min: float = 0.4   # 二重線の 2 本の間隔 [m]
    double_line_gap_max: float = 1.1


@dataclass
class TrackedLines:
    lines: Dict[str, Optional[LineFit]] = field(default_factory=dict)  # 役割 -> 線 (補完含む)
    detected: Dict[str, bool] = field(default_factory=dict)            # 役割 -> 実検出か
    offsets: Dict[str, float] = field(default_factory=dict)            # 役割 -> x_ref での横位置
    lane_widths: Dict[str, float] = field(default_factory=dict)        # "left"/"right" 側の中央線<->境界線距離
    reanchored: bool = False                                           # このフレームで役割を付け直した

    @property
    def num_detected(self) -> int:
        return sum(1 for v in self.detected.values() if v)


class LineTracker:
    def __init__(self, params: LineTrackerParams = LineTrackerParams()):
        self.p = params
        self.reset()

    def reset(self):
        w = self.p.lane_width_init
        self.lane_w = {"left": w, "right": w}
        self._set_center(-self.p.init_offset)   # 中央線は車両から見て -init_offset の位置
        self.lost_frames = 0
        self._anchor_shift: Optional[float] = None
        self._anchor_count = 0

    def _set_center(self, center: float):
        """中央線の横位置を与えて, 左右の境界を追跡中の道幅で並べ直す."""
        self.offsets = {"left": center + self.lane_w["left"], "center": center,
                        "right": center - self.lane_w["right"]}

    def seed_lane_position(self, lane_coordinate: float):
        """車両のレーン座標 F (左白線=0, 中央線=3, 右白線=6; 6 レーン走行の横位置) から線の位置を置き直す."""
        F = lane_coordinate
        w = self.lane_w["left"] if F <= 3.0 else self.lane_w["right"]
        self._set_center((F - 3.0) * w / 3.0)
        self._anchor_shift, self._anchor_count = None, 0

    def update(self, fits: List[LineFit]) -> TrackedLines:
        p = self.p
        fits = [f for f in fits if f is not None]
        meas = [float(f.y_at(p.x_ref)) for f in fits]
        heads = [f.heading_at(p.x_ref) for f in fits]

        assignment = self._assign(meas, heads)
        anchored = self._reanchor(meas, heads, [f.x_min for f in fits])
        if anchored is not None:
            assignment = anchored
        self._prefer_inner_boundaries(assignment, meas, heads)
        detected = {r: assignment.get(r) is not None for r in ROLES}

        if not any(detected.values()):
            self.lost_frames += 1
            if self.lost_frames >= p.lost_reset_frames:
                # 中央線の上とは仮定しない: 最後の横位置 (中央線の位置) を保って並びと道幅だけ戻す
                center = self.offsets["center"]
                self.lane_w = {"left": p.lane_width_init, "right": p.lane_width_init}
                self._set_center(center)
                self.lost_frames = 0
            return TrackedLines({r: None for r in ROLES}, detected, dict(self.offsets), dict(self.lane_w))
        self.lost_frames = 0

        lines: Dict[str, Optional[LineFit]] = {r: None for r in ROLES}
        for r in ROLES:
            idx = assignment.get(r)
            if idx is not None:
                lines[r] = fits[idx]
                self.offsets[r] = meas[idx]

        # 道幅の更新 (隣り合う 2 本が同時に見えたときだけ)
        for side, a, b in (("left", "left", "center"), ("right", "center", "right")):
            if detected[a] and detected[b]:
                w = self.offsets[a] - self.offsets[b]
                if p.lane_width_min <= w <= p.lane_width_max:
                    self.lane_w[side] += p.width_alpha * (w - self.lane_w[side])

        # 見えなかった線を補完 (中央から優先的に埋める)
        wl, wr = self.lane_w["left"], self.lane_w["right"]
        if lines["center"] is None:
            if lines["left"] is not None:
                lines["center"] = lines["left"].shifted(-wl)
            elif lines["right"] is not None:
                lines["center"] = lines["right"].shifted(+wr)
        if lines["left"] is None and lines["center"] is not None:
            lines["left"] = lines["center"].shifted(+wl)
        if lines["right"] is None and lines["center"] is not None:
            lines["right"] = lines["center"].shifted(-wr)
        for r in ROLES:
            if not detected[r] and lines[r] is not None:
                self.offsets[r] = float(lines[r].y_at(p.x_ref))

        return TrackedLines(lines, detected, dict(self.offsets), dict(self.lane_w), anchored is not None)

    def _assign(self, meas: List[float], heads: List[float]) -> Dict[str, Optional[int]]:
        """検出線 -> 役割の割り当て (モジュール docstring の条件で総当たり)."""
        p = self.p
        gate = min(self.lane_w.values()) * p.gate_ratio
        # 役割 index の差 -> 期待される横間隔 (0-1: 左幅, 1-2: 右幅, 0-2: 両方)
        spacing = {(0, 1): self.lane_w["left"], (1, 2): self.lane_w["right"],
                   (0, 2): self.lane_w["left"] + self.lane_w["right"]}
        best: Dict[str, Optional[int]] = {}
        best_score = (-1, 0.0)
        n = len(meas)
        # 各役割に「検出線 index or None」を割り当てる全組み合わせ (最大 5^3 通り)
        for combo in itertools.product([None] + list(range(n)), repeat=3):
            used = [c for c in combo if c is not None]
            if len(used) != len(set(used)):
                continue
            ok = True
            cost = 0.0
            for r, c in zip(ROLES, combo):
                if c is None:
                    continue
                d = abs(meas[c] - self.offsets[r])
                if d > gate:
                    ok = False
                    break
                cost += d
            if not ok:
                continue
            ys = [meas[c] for c in combo if c is not None]
            if any(ys[i] <= ys[i + 1] for i in range(len(ys) - 1)):
                continue  # 左 > 中央 > 右 の順序に反する
            if not self._consistent(combo, meas, heads, spacing):
                continue
            score = (len(used), -cost)
            if score > best_score:
                best_score = score
                best = dict(zip(ROLES, combo))
        return best

    def _reanchor(self, meas: List[float], heads: List[float],
                  x_mins: List[float]) -> Optional[Dict[str, Optional[int]]]:
        """3 本 (または左右の境界 2 本) が道幅どおりに揃って見えるのに, 追跡中の位置とゲート以上ずれて
        いれば (= 割り当てが線 1 本ぶんずれている), それが anchor_frames 続いた時点でその割り当てを返す."""
        p = self.p
        gate = min(self.lane_w.values()) * p.gate_ratio
        spacing = {(0, 1): self.lane_w["left"], (1, 2): self.lane_w["right"],
                   (0, 2): self.lane_w["left"] + self.lane_w["right"]}
        candidates = []  # (優先度, 割り当て, 中央線の位置のずれ)
        n = len(meas)
        for combo in itertools.product([None] + list(range(n)), repeat=3):
            left, center, right = combo
            if left is None or right is None:
                continue  # 左右の境界が両方あるときだけ役割が一意に決まる
            used = [c for c in combo if c is not None]
            if len(used) != len(set(used)) or any(x_mins[c] > p.anchor_max_x_min for c in used):
                continue
            ys = [meas[c] for c in used]
            if any(ys[i] <= ys[i + 1] for i in range(len(ys) - 1)):
                continue
            if not self._consistent(combo, meas, heads, spacing, p.anchor_tolerance, p.anchor_heading_diff):
                continue
            c_new = meas[center] if center is not None else meas[left] - self.lane_w["left"]
            candidates.append((0 if center is not None else 1, dict(zip(ROLES, combo)), c_new - self.offsets["center"]))
        if not candidates and p.use_double_line:
            for i in range(n):
                for k in range(n):
                    near, far = meas[i], meas[k]
                    # 同じ側 (車をまたがない) に平行に並び, i の方が車に近い
                    if near * far <= 0 or abs(near) < 0.2 or not abs(near) < abs(far) \
                            or not p.double_line_gap_min <= abs(far - near) <= p.double_line_gap_max \
                            or max(x_mins[i], x_mins[k]) > p.anchor_max_x_min \
                            or abs(heads[i] - heads[k]) > p.anchor_heading_diff:
                        continue
                    role = "right" if near < 0 else "left"
                    c_new = near + self.lane_w["right"] if role == "right" else near - self.lane_w["left"]
                    candidates.append((2, {"left": None, "center": None, "right": None, role: i},
                                       c_new - self.offsets["center"]))
        best = None
        if candidates:
            _, assignment, shift = min(candidates, key=lambda c: (c[0], abs(c[2])))  # 3 本 > 左右境界 > 二重線, 次に今に近い
            best = (assignment, shift)
        if best is None or abs(best[1]) <= gate:
            self._anchor_shift, self._anchor_count = None, 0  # 根拠なし, または追跡と一致している
            return None
        assignment, shift = best
        if self._anchor_shift is not None and abs(shift - self._anchor_shift) <= gate:
            self._anchor_count += 1
        else:
            self._anchor_count = 1
        self._anchor_shift = shift
        if self._anchor_count < p.anchor_frames:
            return None
        self._anchor_shift, self._anchor_count = None, 0
        return assignment

    def _consistent(self, combo, meas, heads, spacing, tolerance: Optional[float] = None,
                    heading_diff: Optional[float] = None) -> bool:
        """同時に割り当てる線どうしの間隔が道幅と整合し, 向きが揃っているか."""
        p = self.p
        tol = p.width_tolerance if tolerance is None else tolerance
        max_head = p.max_heading_diff if heading_diff is None else heading_diff
        roles = [k for k in range(3) if combo[k] is not None]
        for a_i in range(len(roles)):
            for b_i in range(a_i + 1, len(roles)):
                a, b = roles[a_i], roles[b_i]
                ia, ib = combo[a], combo[b]
                expected = spacing[(a, b)]
                if abs((meas[ia] - meas[ib]) - expected) > tol * expected:
                    return False
                if abs(heads[ia] - heads[ib]) > max_head:
                    return False
        return True

    def _prefer_inner_boundaries(self, assignment, meas, heads):
        """境界に割り当てた線と中央線の間に, 道幅の整合する (中央線から lane_width±tol) 未使用の線があれば付け替える."""
        p = self.p
        c = assignment.get("center")
        if c is None:
            return
        used = {v for v in assignment.values() if v is not None}
        for role, sign in (("left", 1.0), ("right", -1.0)):
            b = assignment.get(role)
            if b is None:
                continue
            w = self.lane_w[role]
            best = None
            for k, m in enumerate(meas):
                if k in used:
                    continue
                d = sign * (m - meas[c])                 # 中央線から外向きの距離
                if 0.0 < d < sign * (meas[b] - meas[c]) and abs(d - w) <= p.width_tolerance * w \
                        and abs(heads[k] - heads[c]) <= p.max_heading_diff:
                    if best is None or d < sign * (meas[best] - meas[c]):
                        best = k
            if best is not None:
                used.discard(b)
                used.add(best)
                assignment[role] = best
