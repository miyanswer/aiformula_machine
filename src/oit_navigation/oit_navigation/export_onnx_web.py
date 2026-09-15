#!/usr/bin/env python3
"""
export_onnx_web.py - Export the YOLOP lane-line segmentation head to ONNX for
in-browser inference (see web_simulator/js/lane_model_detector.js). This lets
the web simulator run the same white-line detection yolop_lane_detector.py
runs on the real vehicle, directly in the browser via onnxruntime-web.

Only the lane-line segmentation head is exported: the object-detection head
(vehicles/traffic lights) has no consumer in the web simulator, so it is
dropped from the wrapped forward() and pruned from the ONNX graph by the
exporter's dead-code elimination.

The input shape is a fixed 640x640 square (not dynamic, not letterboxed):
the web simulator preprocesses its onboard-camera capture with the same
"crop_bottom" ROI as yolop_lane_detector.py (drop the top top_cut_ratio
fraction of rows, keeping the road-relevant bottom portion), then resizes
that crop directly to 640x640 -- see js/lane_model_detector.js.

Usage:
    python3 export_onnx_web.py \\
        --weights /aiformula_machine/models/honda_shihou_finetuned_best.pth \\
        --output /aiformula_machine/web_simulator/models/honda_shihou_finetuned.onnx
"""

import argparse
import os
import sys
from pathlib import Path

import torch
import torch.nn as nn

_THIS_DIR = Path(__file__).resolve().parent
_YOLOP_DIR = _THIS_DIR / "yolop"
if _YOLOP_DIR.exists() and str(_YOLOP_DIR) not in sys.path:
    sys.path.insert(0, str(_YOLOP_DIR))

from lib.config import cfg  # noqa: E402
from lib.models import get_net  # noqa: E402

# Matches web_simulator's crop_bottom preprocessing target size (square,
# no letterbox padding -- see module docstring above).
DEFAULT_INPUT_SIZE = 640


class LaneOnlyExportWrapper(nn.Module):
    def __init__(self, model: nn.Module):
        super().__init__()
        self.model = model

    def forward(self, x):
        _det_out, _da_seg, ll_seg = self.model(x)
        return ll_seg


def load_model(weights_path: str) -> nn.Module:
    model = get_net(cfg)
    checkpoint = torch.load(weights_path, map_location="cpu")
    state_dict = checkpoint["state_dict"] if isinstance(checkpoint, dict) and "state_dict" in checkpoint else checkpoint
    model.load_state_dict(state_dict)
    model.eval()
    return model


def export_onnx(model: nn.Module, onnx_path: str, size: int) -> None:
    wrapper = LaneOnlyExportWrapper(model).eval()
    dummy_input = torch.zeros(1, 3, size, size, dtype=torch.float32)
    os.makedirs(os.path.dirname(onnx_path) or ".", exist_ok=True)

    export_kwargs = dict(
        input_names=["input"],
        output_names=["ll_seg"],
        opset_version=12,
        do_constant_folding=True,
    )
    try:
        # See export_tensorrt.py's identical fallback: PyTorch >= 2.5 defaults
        # to the dynamo exporter, which this older research-repo model doesn't
        # satisfy; force the legacy TorchScript-tracing exporter.
        torch.onnx.export(wrapper, dummy_input, onnx_path, dynamo=False, **export_kwargs)
    except TypeError:
        torch.onnx.export(wrapper, dummy_input, onnx_path, **export_kwargs)
    print(f"[export_onnx_web] Wrote ONNX graph: {onnx_path} (input: 1x3x{size}x{size})")


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", default="/aiformula_machine/models/honda_shihou_finetuned_best.pth")
    parser.add_argument("--output", default="/aiformula_machine/web_simulator/models/honda_shihou_finetuned.onnx")
    parser.add_argument("--size", type=int, default=DEFAULT_INPUT_SIZE, help="Square input side length")
    parsed = parser.parse_args(args=args)

    model = load_model(parsed.weights)
    export_onnx(model, parsed.output, parsed.size)


if __name__ == "__main__":
    main()
