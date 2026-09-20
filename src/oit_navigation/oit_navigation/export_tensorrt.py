#!/usr/bin/env python3
"""
export_tensorrt.py - Export the trained YOLOP PyTorch weights to a TensorRT engine
for real-hardware (Jetson) inference acceleration.

Run this ON THE TARGET DEVICE. A TensorRT engine is tied to the exact GPU
architecture and TensorRT version it was built with - an engine built on a
dev PC (or a different Jetson/TensorRT version) will not load elsewhere.
Requires the NVIDIA TensorRT Python bindings and PyCUDA, which ship with
JetPack / the TensorRT SDK - they are intentionally not a dependency of this
package, since development on this project happens mostly on machines
without an NVIDIA GPU at all (see yolop_lane_backend.py's PyTorch fallback).

Usage (on the Jetson, with the workspace sourced):
    ros2 run oit_navigation export_tensorrt -- \\
        --weights /aiformula_machine/models/honda_shihou_finetuned_best.pth \\
        --output /aiformula_machine/models/honda_shihou_finetuned_best.engine --fp16

    # INT8 needs representative calibration images from real track footage -
    # accuracy depends entirely on how representative these images are:
    ros2 run oit_navigation export_tensorrt -- \\
        --weights ... --output ... --int8 --calib-images-dir /path/to/sample_frames
"""

import argparse
import glob
import os
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

_THIS_DIR = Path(__file__).resolve().parent
_YOLOP_DIR = _THIS_DIR / "yolop"
if _YOLOP_DIR.exists() and str(_YOLOP_DIR) not in sys.path:
    sys.path.insert(0, str(_YOLOP_DIR))

from lib.config import cfg  # noqa: E402
from lib.models import get_net  # noqa: E402

INPUT_SIZE = 640


class YOLOPExportWrapper(nn.Module):
    """Flattens YOLOP's nested output to exactly the two tensors this project
    uses at inference time (see yolop_lane_backend.py's _run_inference):
    raw detection predictions and the lane-line segmentation logits. The
    drivable-area segmentation head and the training-only feature maps are
    dropped since nothing downstream consumes them."""

    def __init__(self, model: nn.Module):
        super().__init__()
        self.model = model

    def forward(self, x):
        det_out, _da_seg, ll_seg = self.model(x)
        raw_detections, _features = det_out
        return raw_detections, ll_seg


def load_model(weights_path: str) -> nn.Module:
    model = get_net(cfg)
    checkpoint = torch.load(weights_path, map_location="cpu")
    state_dict = checkpoint["state_dict"] if isinstance(checkpoint, dict) and "state_dict" in checkpoint else checkpoint
    model.load_state_dict(state_dict)
    model.eval()
    return model


def export_onnx(model: nn.Module, onnx_path: str) -> None:
    wrapper = YOLOPExportWrapper(model).eval()
    dummy_input = torch.zeros(1, 3, INPUT_SIZE, INPUT_SIZE, dtype=torch.float32)
    export_kwargs = dict(
        input_names=["input"],
        output_names=["raw_detections", "ll_seg"],
        opset_version=12,
        do_constant_folding=True,
    )
    try:
        # PyTorch >= 2.5 defaults to its newer dynamo-based exporter, which
        # needs the separate `onnxscript` package and is far stricter about
        # pure torch.export-compatible control flow than this YOLOP model
        # (adapted from an older research repo) satisfies. The legacy
        # TorchScript-tracing exporter (dynamo=False) handles it correctly.
        torch.onnx.export(wrapper, dummy_input, onnx_path, dynamo=False, **export_kwargs)
    except TypeError:
        # Older PyTorch (no dynamo kwarg at all) always used the legacy exporter.
        torch.onnx.export(wrapper, dummy_input, onnx_path, **export_kwargs)
    print(f"[export_tensorrt] Wrote ONNX graph: {onnx_path}")


def _load_calibration_batches(images_dir: str):
    """Yields (1,3,640,640) float32 arrays from a directory of sample track images,
    preprocessed identically to lane_detector (backend=yolop)'s letterbox+normalize path
    (plain resize here, since exact letterbox padding only shifts a few border
    pixels and does not meaningfully change INT8 calibration statistics)."""
    import cv2

    paths = sorted(glob.glob(os.path.join(images_dir, "*.jpg")) + glob.glob(os.path.join(images_dir, "*.png")))
    if not paths:
        raise FileNotFoundError(f"No calibration images (*.jpg/*.png) found in {images_dir}")

    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std = np.array([0.229, 0.224, 0.225], dtype=np.float32)

    for path in paths:
        img = cv2.imread(path)
        if img is None:
            continue
        img = cv2.resize(img, (INPUT_SIZE, INPUT_SIZE))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        img = (img - mean) / std
        img = np.transpose(img, (2, 0, 1))[None, ...].astype(np.float32)
        yield np.ascontiguousarray(img)


class _EntropyCalibrator:
    """Minimal INT8 entropy calibrator. INT8 accuracy depends entirely on how
    representative the calibration images are of actual deployment conditions -
    always prefer --calib-images-dir with real track footage over the
    synthetic-noise fallback used when none is given."""

    def __init__(self, trt, images_dir: str, cache_path: str):
        import pycuda.driver as cuda
        import pycuda.autoinit  # noqa: F401

        self._cuda = cuda
        self.trt = trt
        self.cache_path = cache_path

        if images_dir:
            self.batches = list(_load_calibration_batches(images_dir))
        else:
            print(
                "[export_tensorrt] WARNING: no --calib-images-dir given; calibrating "
                "on synthetic random noise. INT8 accuracy will likely be poor - pass "
                "real sample frames if at all possible."
            )
            rng = np.random.default_rng(0)
            self.batches = [
                rng.standard_normal((1, 3, INPUT_SIZE, INPUT_SIZE)).astype(np.float32) for _ in range(20)
            ]
        self.index = 0
        self.device_input = cuda.mem_alloc(self.batches[0].nbytes)

    def get_batch_size(self):
        return 1

    def get_batch(self, names):
        if self.index >= len(self.batches):
            return None
        batch = np.ascontiguousarray(self.batches[self.index])
        self._cuda.memcpy_htod(self.device_input, batch)
        self.index += 1
        return [int(self.device_input)]

    def read_calibration_cache(self):
        if os.path.exists(self.cache_path):
            with open(self.cache_path, "rb") as f:
                return f.read()
        return None

    def write_calibration_cache(self, cache):
        with open(self.cache_path, "wb") as f:
            f.write(cache)


def build_engine(onnx_path: str, engine_path: str, fp16: bool, int8: bool, calib_images_dir: str) -> None:
    import tensorrt as trt

    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)

    with open(onnx_path, "rb") as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print(parser.get_error(i))
            raise RuntimeError("Failed to parse ONNX graph for TensorRT")

    config = builder.create_builder_config()
    try:
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)  # TensorRT >= 8.4
    except AttributeError:
        config.max_workspace_size = 1 << 30  # TensorRT < 8.4

    if fp16:
        if not builder.platform_has_fast_fp16:
            print("[export_tensorrt] WARNING: platform reports no fast-FP16 support; building anyway.")
        config.set_flag(trt.BuilderFlag.FP16)

    calibrator = None
    if int8:
        if not builder.platform_has_fast_int8:
            print("[export_tensorrt] WARNING: platform reports no fast-INT8 support; building anyway.")
        config.set_flag(trt.BuilderFlag.INT8)
        calibrator = _EntropyCalibrator(trt, calib_images_dir, engine_path + ".calibration_cache")
        config.int8_calibrator = calibrator

    if hasattr(builder, "build_serialized_network"):  # TensorRT >= 8.0
        serialized_engine = builder.build_serialized_network(network, config)
        if serialized_engine is None:
            raise RuntimeError("TensorRT engine build failed")
        with open(engine_path, "wb") as f:
            f.write(serialized_engine)
    else:  # TensorRT < 8.0
        engine = builder.build_engine(network, config)
        if engine is None:
            raise RuntimeError("TensorRT engine build failed")
        with open(engine_path, "wb") as f:
            f.write(engine.serialize())

    print(f"[export_tensorrt] Wrote TensorRT engine: {engine_path}")


def get_gpu_device_tag(device_id: int = 0) -> str:
    """Returns a sanitized identifier for the current GPU and compute capability,
    e.g. 'rtx4090_sm89' or 'orin_sm87' or 'cuda0_sm86'."""
    if not torch.cuda.is_available():
        return "cpu"
    try:
        raw_name = torch.cuda.get_device_name(device_id)
        # Clean up common brand prefixes
        clean = raw_name.replace("NVIDIA", "").replace("GeForce", "").replace("Laptop GPU", "")
        clean = "".join(c if c.isalnum() else "_" for c in clean).strip("_").lower()
        while "__" in clean:
            clean = clean.replace("__", "_")
        major, minor = torch.cuda.get_device_capability(device_id)
        return f"{clean}_sm{major}{minor}"
    except Exception:
        return f"cuda{device_id}"


def export_engine_for_current_gpu(
    weights_path: str,
    engine_path: str,
    fp16: bool = True,
    int8: bool = False,
    calib_images_dir: str = "",
) -> str:
    """Exports and builds a TensorRT engine directly for the current GPU,
    returning the engine_path on success."""
    model = load_model(weights_path)
    onnx_path = os.path.splitext(engine_path)[0] + ".onnx"
    export_onnx(model, onnx_path)
    build_engine(onnx_path, engine_path, fp16=fp16, int8=int8, calib_images_dir=calib_images_dir)
    return engine_path


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", required=True, help="Path to the trained YOLOP .pth weights")
    parser.add_argument("--output", default="", help="Path to write the .engine file to (auto-named by GPU if omitted)")
    parser.add_argument("--onnx-out", default=None, help="Optional path to keep the intermediate .onnx file")
    parser.add_argument("--fp16", action="store_true", help="Build an FP16 engine (recommended default)")
    parser.add_argument("--int8", action="store_true", help="Build an INT8 engine (needs calibration data)")
    parser.add_argument("--calib-images-dir", default="", help="Directory of representative track images for INT8 calibration")
    parsed = parser.parse_args(args=args)

    if parsed.int8 and parsed.fp16:
        parser.error("--fp16 and --int8 are mutually exclusive (TensorRT builds one precision mode at a time)")

    output_path = parsed.output
    if not output_path:
        gpu_tag = get_gpu_device_tag(0)
        base_name = os.path.splitext(parsed.weights)[0]
        output_path = f"{base_name}_{gpu_tag}.engine"

    export_engine_for_current_gpu(
        weights_path=parsed.weights,
        engine_path=output_path,
        fp16=parsed.fp16 or not parsed.int8,
        int8=parsed.int8,
        calib_images_dir=parsed.calib_images_dir,
    )


if __name__ == "__main__":
    main()
