"""
白線セグメンテーションマスク (YOLOP の ll_seg など) -> 白線ごとの画像点列.

UFLD は「車線スロットごとの点列」を直接出すが, YOLOP はピクセル単位のマスクしか出さない.
そこで画像下端から上へ行ごとに白線ピクセルの塊 (run) の中心を取り, 前の行の点と近いもの同士を
つないで 1 本ずつの点列 (chain) にする. 出力形式は ufld/model.py decode_lanes() と同じ
[{"u": ..., "v": ..., "conf": ...}, ...] なので, 以降の地面投影・フィット・役割割り当ては共通.

    - 行は下 (手前) から上 (奥) へ row_step_ratio * 高さ 間隔でサンプルする.
    - 太すぎる run (横断歩道・停止線など路面標示) は捨てる.
    - 連結ゲートは画像幅比で指定 (奥ほど白線間の画素距離が縮むので上に行くほど狭める).
"""

from dataclasses import dataclass
from typing import Dict, List

import numpy as np


@dataclass
class MaskLinesParams:
    top_ratio: float = 0.45          # これより上 (空・遠景) は使わない (YOLOP の top_cut_ratio と同じ)
    row_step_ratio: float = 1 / 72   # 行サンプル間隔 (高さ比)
    min_run_px_ratio: float = 0.002  # これより細い run はノイズ (幅比)
    max_run_width_ratio: float = 0.12  # これより太い run は白線ではない路面標示 (画像下端での幅比, 上に行くほど縮める)
    gate_ratio: float = 0.10         # 行間の連結ゲート (画像下端での幅比)
    max_gap_rows: int = 4            # これ以上の行で途切れたら別の線
    min_points: int = 4


def extract_mask_lines(mask: np.ndarray, p: MaskLinesParams = MaskLinesParams()) -> List[Dict[str, np.ndarray]]:
    h, w = mask.shape[:2]
    top = int(h * p.top_ratio)
    step = max(1, int(round(h * p.row_step_ratio)))
    rows = list(range(h - 1, top, -step))
    chains = []   # {"u": [...], "v": [...], "last_i": row index}
    for ri, v in enumerate(rows):
        depth = (v - top) / max(h - 1 - top, 1)          # 1: 下端, 0: top
        scale = 0.4 + 0.6 * depth                        # 奥ほど小さく
        max_run = p.max_run_width_ratio * w * scale
        gate = max(3.0, p.gate_ratio * w * scale)
        row = mask[v] > 0
        if not row.any():
            continue
        d = np.diff(np.concatenate([[0], row.astype(np.int8), [0]]))
        starts, ends = np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]
        centers = [(s + e - 1) / 2.0 for s, e in zip(starts, ends)
                   if p.min_run_px_ratio * w <= (e - s) <= max_run]
        used = set()
        # 既存 chain ごとに最近傍の run をつなぐ (近い順)
        cands = []
        for ci, ch in enumerate(chains):
            if ri - ch["last_i"] > p.max_gap_rows:
                continue
            pred = ch["u"][-1]
            if len(ch["u"]) >= 2:   # 直前 2 点の傾きで外挿
                pred += (ch["u"][-1] - ch["u"][-2]) * (ri - ch["last_i"])
            for k, c in enumerate(centers):
                dist = abs(c - pred)
                if dist <= gate:
                    cands.append((dist, ci, k))
        cands.sort()
        taken_chain = set()
        for dist, ci, k in cands:
            if ci in taken_chain or k in used:
                continue
            chains[ci]["u"].append(centers[k])
            chains[ci]["v"].append(float(v))
            chains[ci]["last_i"] = ri
            taken_chain.add(ci)
            used.add(k)
        for k, c in enumerate(centers):
            if k not in used:
                chains.append({"u": [c], "v": [float(v)], "last_i": ri})

    lanes = []
    for ch in chains:
        if len(ch["u"]) < p.min_points:
            continue
        u = np.asarray(ch["u"][::-1])
        v = np.asarray(ch["v"][::-1])     # 上 -> 下 (decode_lanes と同じ並び)
        lanes.append({"u": u, "v": v, "conf": np.ones_like(u)})
    return lanes
