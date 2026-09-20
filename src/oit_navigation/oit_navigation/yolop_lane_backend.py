"""
yolop_lane_backend.py - YOLOP の白線セグメンテーション (ll_seg) だけを取り出す推論ラッパー (ROS 非依存).

実機 (Jetson) は GitHub から clone したリポジトリで走らせるため, git 管理されている
models/honda_shihou_finetuned_best.pth (YOLOP) を白線検出に使う. lane_detector ノードの
backend=yolop で使われ, 出力マスクは lane_nav/mask_lines.py で白線ごとの点列に変換される.

    - 前処理: 画像上部 (top_cut_ratio) を黒塗り (roi_mode=mask_top) または切り落とし (crop_bottom),
      640x640 に letterbox, BGR のまま ImageNet mean/std で正規化 (学習時と同じ).
    - 推論: PyTorch, または use_tensorrt=True なら TensorRT エンジン (Jetson).
      エンジンが無ければ export_tensorrt.py で GPU 用にその場でビルドしてキャッシュする.
      TensorRT が使えない環境では自動的に PyTorch にフォールバック.
    - 物体検出ヘッド / 走行可能領域ヘッドの出力は使わない.
"""

import os
import sys
from pathlib import Path
from typing import Callable, Optional

import cv2
import numpy as np

from common_python.workspace_paths import resolve_workspace_asset

_LOCAL_YOLOP_DIR = Path(__file__).resolve().parent / "yolop"
if _LOCAL_YOLOP_DIR.exists() and str(_LOCAL_YOLOP_DIR) not in sys.path:
    sys.path.insert(0, str(_LOCAL_YOLOP_DIR))

INPUT_SIZE = 640


class YOLOPLaneModel:
    def __init__(
        self,
        weight_path: str,
        device: str = "cpu",
        roi_mode: str = "mask_top",
        top_cut_ratio: float = 0.45,
        norm_mean=(0.485, 0.456, 0.406),
        norm_std=(0.229, 0.224, 0.225),
        use_tensorrt: bool = False,
        tensorrt_engine_path: str = "",
        log: Callable[[str], None] = print,
    ):
        import torch

        self._torch = torch
        self.log = log
        self.roi_mode = roi_mode
        self.top_cut_ratio = top_cut_ratio
        self.mean = np.asarray(norm_mean, np.float32)
        self.std = np.asarray(norm_std, np.float32)
        self.device = self._resolve_device(str(device))
        self.weight_path = resolve_workspace_asset(weight_path)
        if not os.path.exists(self.weight_path):
            raise FileNotFoundError(f"YOLOP の重みが見つかりません: {weight_path}")

        self.trt_runner = None
        if use_tensorrt:
            self.trt_runner = self._init_tensorrt(tensorrt_engine_path)
            if self.trt_runner is None:
                self.log("[YOLOP] TensorRT が使えないため PyTorch で推論します")
        self.net = None
        self.use_half = False
        if self.trt_runner is None:
            from lib.config import cfg
            from lib.models import get_net
            net = get_net(cfg)
            ckpt = torch.load(self.weight_path, map_location="cpu", weights_only=False)
            state = ckpt["state_dict"] if isinstance(ckpt, dict) and "state_dict" in ckpt else ckpt
            net.load_state_dict(state)
            self.use_half = self.device.type == "cuda"
            net = net.to(self.device)
            self.net = (net.half() if self.use_half else net.float()).eval()

    @property
    def backend_name(self) -> str:
        return "TensorRT" if self.trt_runner is not None else f"PyTorch({self.device})"

    def _resolve_device(self, device: str):
        torch = self._torch
        if device in ("", "cpu"):
            return torch.device("cpu")
        if device == "mps":
            return torch.device("mps" if torch.backends.mps.is_available() else "cpu")
        if torch.cuda.is_available():
            return torch.device(f"cuda:{device}" if device.isdigit() else "cuda")
        return torch.device("cpu")

    def _init_tensorrt(self, engine_path: str):
        torch = self._torch
        if not torch.cuda.is_available():
            return None
        try:
            from oit_navigation.export_tensorrt import get_gpu_device_tag, export_engine_for_current_gpu
            gpu_tag = get_gpu_device_tag(0)
        except Exception:  # noqa: BLE001
            export_engine_for_current_gpu, gpu_tag = None, "cuda0"
        if engine_path:
            engine_path = resolve_workspace_asset(engine_path)
        else:
            base = os.path.splitext(self.weight_path)[0]
            engine_path = f"{base}_{gpu_tag}.engine"
            if not os.path.exists(engine_path) and os.path.exists(base + ".engine"):
                engine_path = base + ".engine"
        if not os.path.exists(engine_path):
            if export_engine_for_current_gpu is None:
                return None
            self.log(f"[YOLOP] TensorRT エンジンをビルドします ({gpu_tag}) -> {engine_path}")
            try:
                export_engine_for_current_gpu(weights_path=self.weight_path, engine_path=engine_path, fp16=True)
            except Exception as e:  # noqa: BLE001
                self.log(f"[YOLOP] エンジンのビルドに失敗: {e}")
                return None
        try:
            from oit_navigation.utils.tensorrt_runtime import TensorRTYOLOPRunner
            runner = TensorRTYOLOPRunner(engine_path)
            self.log(f"[YOLOP] TensorRT エンジンを読み込みました: {engine_path}")
            return runner
        except Exception as e:  # noqa: BLE001
            self.log(f"[YOLOP] TensorRT エンジンの読み込みに失敗: {e}")
            return None

    def infer_mask(self, bgr: np.ndarray) -> np.ndarray:
        """白線マスク (元画像と同じサイズ, 0/1 uint8) を返す."""
        torch = self._torch
        h, w = bgr.shape[:2]
        cut = int(h * self.top_cut_ratio)
        if self.roi_mode == "crop_bottom":
            proc = bgr[cut:]
        elif self.roi_mode == "mask_top":
            proc = bgr.copy()
            proc[:cut] = 0
        else:
            proc = bgr
        ph, pw = proc.shape[:2]

        # letterbox (縦横比維持で 640 に収め, 余白は 114 で埋める)
        r = min(INPUT_SIZE / ph, INPUT_SIZE / pw)
        nh, nw = int(round(ph * r)), int(round(pw * r))
        top = (INPUT_SIZE - nh) // 2
        left = (INPUT_SIZE - nw) // 2
        canvas = np.full((INPUT_SIZE, INPUT_SIZE, 3), 114, np.uint8)
        canvas[top:top + nh, left:left + nw] = cv2.resize(proc, (nw, nh), interpolation=cv2.INTER_AREA)
        x = (canvas.astype(np.float32) / 255.0 - self.mean) / self.std   # BGR のまま (学習時と同じ)
        x = torch.from_numpy(np.ascontiguousarray(x.transpose(2, 0, 1)[None]))

        if self.trt_runner is not None:
            _det, ll = self.trt_runner.infer(x.to("cuda"))
        else:
            x = x.to(self.device)
            x = x.half() if self.use_half else x
            with torch.inference_mode():
                _det, _da, ll = self.net(x)
        ll = ll[:, :, top:top + nh, left:left + nw].float()
        seg = (ll[0, 1] > ll[0, 0]).to(torch.uint8).cpu().numpy()
        sub = cv2.resize(seg, (pw, ph), interpolation=cv2.INTER_NEAREST)

        if self.roi_mode == "crop_bottom":
            mask = np.zeros((h, w), np.uint8)
            mask[cut:] = sub
        else:
            mask = sub
            mask[:cut] = 0
        return mask
