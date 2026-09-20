"""
UFLD v1 (Ultra-Fast-Lane-Detection, cfzd/Ultra-Fast-Lane-Detection) の推論実装.

models/ufld_honda_finetuned_best.pth の中身は {"model": state_dict, "epoch", "best_acc"}
で, state_dict の形状から以下が読み取れる:
    - backbone: ResNet18 (layer1-4 が各 2 BasicBlock)
    - pool:     Conv2d(512 -> 8, 1x1),  cls.0: Linear(1800 -> 2048)
      -> 特徴マップ 9x25 = 入力 288x800
    - cls.2:    Linear(2048 -> 22624) = (griding_num + 1) * cls_num_per_lane * num_lanes
      -> 101 * 56 * 4 (TuSimple 設定: griding_num=100, row anchor 56 本, 車線スロット 4)

この形状判定を infer_config_from_state_dict() で行うので, CULane 設定 (200, 18, 4) や
ResNet34 で学習し直した重みに差し替えてもそのまま読み込める.

出力の解釈 (公式 demo.py と同じ):
    out: (griding_num + 1, cls_num_per_lane, num_lanes)
    - 各 row anchor (画像の横ライン) ごとに, 列方向 griding_num 個のセルのどこに白線があるかの分類.
      最後のクラス (index = griding_num) は「その行に白線なし」.
    - 列位置はセル確率の期待値 (softmax 加重平均) でサブセル精度にする.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

# 学習時の入力解像度 (UFLD v1 は全データセット共通で 288x800)
UFLD_INPUT_H = 288
UFLD_INPUT_W = 800

# 公式 configs の row anchor (288 px 高さの入力画像上の y 座標)
TUSIMPLE_ROW_ANCHOR = list(range(64, 288, 4))  # 56 本
CULANE_ROW_ANCHOR = [121, 131, 141, 150, 160, 170, 180, 189, 199, 209,
                     219, 228, 238, 248, 258, 267, 277, 287]  # 18 本

_IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


@dataclass
class UFLDConfig:
    backbone: str = "18"
    griding_num: int = 100
    cls_num_per_lane: int = 56
    num_lanes: int = 4
    row_anchor: List[int] = field(default_factory=lambda: list(TUSIMPLE_ROW_ANCHOR))
    input_h: int = UFLD_INPUT_H
    input_w: int = UFLD_INPUT_W


def infer_config_from_state_dict(state_dict: Dict[str, "np.ndarray"]) -> UFLDConfig:
    """state_dict のテンソル形状から backbone / griding_num / row anchor 数 / 車線数を判定する."""
    keys = list(state_dict.keys())

    if any(k.startswith("model.layer1.0.conv3") for k in keys):
        backbone = "50"
    else:
        n_layer3_blocks = len({k.split(".")[2] for k in keys if k.startswith("model.layer3.")})
        backbone = "34" if n_layer3_blocks >= 6 else "18"

    total_dim = int(state_dict["cls.2.weight"].shape[0])

    # 既知の組み合わせを優先 (TuSimple / CULane), 無ければ num_lanes=4 で素因数から推定
    for griding_num, cls_num, row_anchor in (
        (100, 56, TUSIMPLE_ROW_ANCHOR),
        (200, 18, CULANE_ROW_ANCHOR),
    ):
        for num_lanes in (4, 2, 3, 5, 6):
            if (griding_num + 1) * cls_num * num_lanes == total_dim:
                return UFLDConfig(backbone, griding_num, cls_num, num_lanes, list(row_anchor))

    raise ValueError(
        f"UFLD の出力次元 {total_dim} から griding_num / row anchor 数を判定できません. "
        "ufld_griding_num / ufld_cls_num_per_lane / ufld_num_lanes パラメータで明示してください."
    )


def _build_parsing_net(cfg: UFLDConfig):
    """公式 model/model.py の parsingNet (use_aux=False) と同じモジュール構成を組み立てる."""
    import torch
    import torchvision

    class _ResNetBackbone(torch.nn.Module):
        def __init__(self, layers: str):
            super().__init__()
            ctor = {"18": torchvision.models.resnet18,
                    "34": torchvision.models.resnet34,
                    "50": torchvision.models.resnet50}[layers]
            net = ctor(weights=None)
            self.conv1 = net.conv1
            self.bn1 = net.bn1
            self.relu = net.relu
            self.maxpool = net.maxpool
            self.layer1 = net.layer1
            self.layer2 = net.layer2
            self.layer3 = net.layer3
            self.layer4 = net.layer4

        def forward(self, x):
            x = self.maxpool(self.relu(self.bn1(self.conv1(x))))
            x = self.layer1(x)
            x = self.layer2(x)
            x = self.layer3(x)
            return self.layer4(x)

    class _ParsingNet(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.cls_dim = (cfg.griding_num + 1, cfg.cls_num_per_lane, cfg.num_lanes)
            total_dim = int(np.prod(self.cls_dim))
            self.feat_dim = 8 * (cfg.input_h // 32) * (cfg.input_w // 32)
            self.model = _ResNetBackbone(cfg.backbone)
            in_ch = 2048 if cfg.backbone == "50" else 512
            self.pool = torch.nn.Conv2d(in_ch, 8, 1)
            self.cls = torch.nn.Sequential(
                torch.nn.Linear(self.feat_dim, 2048),
                torch.nn.ReLU(),
                torch.nn.Linear(2048, total_dim),
            )

        def forward(self, x):
            fea = self.pool(self.model(x)).view(-1, self.feat_dim)
            return self.cls(fea).view(-1, *self.cls_dim)

    return _ParsingNet()


def decode_lanes(
    out: np.ndarray,
    cfg: UFLDConfig,
    img_w: int,
    img_h: int,
    min_points: int = 3,
) -> List[Optional[Dict[str, np.ndarray]]]:
    """UFLD の生出力 (griding_num+1, cls_num_per_lane, num_lanes) を画像座標の点列に変換する.

    戻り値は車線スロットごとのリスト. 白線なしのスロットは None,
    ありのスロットは {"u": 列座標[px], "v": 行座標[px], "conf": 各点の最大確率} (v 昇順 = 画像上から下).
    """
    griding_num = cfg.griding_num
    logits = out.astype(np.float32)
    logits = logits - logits.max(axis=0, keepdims=True)
    exp = np.exp(logits)
    prob_all = exp / exp.sum(axis=0, keepdims=True)  # (G+1, R, L)

    argmax = prob_all.argmax(axis=0)                 # (R, L)
    prob_cells = prob_all[:-1]                       # "白線なし" クラスを除いた列セル
    cell_sum = prob_cells.sum(axis=0, keepdims=True)
    loc = (prob_cells / np.maximum(cell_sum, 1e-9) * (np.arange(griding_num) + 1)[:, None, None]).sum(axis=0)

    col_sample_w = (cfg.input_w - 1) / (griding_num - 1)
    row_anchor = np.asarray(cfg.row_anchor, dtype=np.float32)

    lanes: List[Optional[Dict[str, np.ndarray]]] = []
    for lane_idx in range(cfg.num_lanes):
        valid = argmax[:, lane_idx] != griding_num
        if int(valid.sum()) < min_points:
            lanes.append(None)
            continue
        rows = np.nonzero(valid)[0]
        u = loc[rows, lane_idx] * col_sample_w * img_w / cfg.input_w - 1.0
        v = row_anchor[rows] * img_h / cfg.input_h - 1.0
        conf = prob_all.max(axis=0)[rows, lane_idx]
        order = np.argsort(v)
        lanes.append({"u": u[order], "v": v[order], "conf": conf[order]})
    return lanes


class UFLDLaneModel:
    """UFLD v1 の重み読み込み・前処理・推論・デコードをまとめたラッパー (ROS 非依存)."""

    def __init__(
        self,
        weight_path: str,
        device: str = "cpu",
        griding_num: int = 0,
        cls_num_per_lane: int = 0,
        num_lanes: int = 0,
        row_anchor: Optional[Sequence[int]] = None,
    ):
        import torch

        self._torch = torch
        ckpt = torch.load(weight_path, map_location="cpu", weights_only=False)
        state_dict = ckpt.get("model", ckpt) if isinstance(ckpt, dict) else ckpt
        # DataParallel で保存された重みの "module." プレフィックスを外す
        state_dict = {k[7:] if k.startswith("module.") else k: v for k, v in state_dict.items()}

        cfg = infer_config_from_state_dict(state_dict)
        # 明示パラメータがあれば自動判定より優先
        if griding_num > 0:
            cfg.griding_num = griding_num
        if cls_num_per_lane > 0:
            cfg.cls_num_per_lane = cls_num_per_lane
        if num_lanes > 0:
            cfg.num_lanes = num_lanes
        if row_anchor:
            cfg.row_anchor = [int(r) for r in row_anchor]
        if len(cfg.row_anchor) != cfg.cls_num_per_lane:
            raise ValueError(
                f"row_anchor の本数 ({len(cfg.row_anchor)}) が cls_num_per_lane ({cfg.cls_num_per_lane}) と一致しません")
        self.cfg = cfg

        self.device = self._resolve_device(device)
        net = _build_parsing_net(cfg)
        # use_aux=True で学習した重みの補助セグメンテーションヘッドは推論に不要なので無視
        net_keys = set(net.state_dict().keys())
        filtered = {k: v for k, v in state_dict.items() if k in net_keys}
        missing = net_keys - set(filtered.keys())
        if missing:
            raise RuntimeError(f"UFLD 重みに必要なキーが不足しています: {sorted(missing)[:5]} ...")
        net.load_state_dict(filtered, strict=True)
        self.net = net.to(self.device).eval()
        self.use_half = self.device.type == "cuda"
        if self.use_half:
            self.net = self.net.half()

    def _resolve_device(self, device: str):
        torch = self._torch
        device = str(device).strip().lower()
        if device in ("", "cpu"):
            return torch.device("cpu")
        if device == "mps":
            return torch.device("mps" if torch.backends.mps.is_available() else "cpu")
        if device.isdigit() or device.startswith("cuda"):
            if torch.cuda.is_available():
                return torch.device(f"cuda:{device}" if device.isdigit() else device)
            return torch.device("cpu")
        return torch.device("cpu")

    def preprocess(self, bgr: np.ndarray) -> "np.ndarray":
        import cv2
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (self.cfg.input_w, self.cfg.input_h), interpolation=cv2.INTER_LINEAR)
        x = (resized.astype(np.float32) / 255.0 - _IMAGENET_MEAN) / _IMAGENET_STD
        return np.ascontiguousarray(x.transpose(2, 0, 1)[None])

    def infer_raw(self, bgr: np.ndarray) -> np.ndarray:
        torch = self._torch
        x = torch.from_numpy(self.preprocess(bgr)).to(self.device)
        if self.use_half:
            x = x.half()
        with torch.inference_mode():
            out = self.net(x)
        return out[0].float().cpu().numpy()

    def detect(self, bgr: np.ndarray, min_points: int = 3) -> List[Optional[Dict[str, np.ndarray]]]:
        h, w = bgr.shape[:2]
        return decode_lanes(self.infer_raw(bgr), self.cfg, w, h, min_points=min_points)
