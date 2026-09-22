#!/usr/bin/env python3
"""
export_cone_onnx.py - cone.pt (Ultralytics YOLO 検出モデル) を
web_simulator 用のONNXへ変換する。

ROSノードではない。export_onnx_web.py (YOLOP) / export_ufld_onnx と同じ、
ブラウザ実行用アセットを書き出すだけのツール。web_simulator/js/cone_detector.js
がonnxruntime-webでこの出力を読み込む。

cone.pt はYOLOPと違い素のUltralytics YOLO検出モデルなので、YOLOPのような
カスタムラッパーは不要 -- Ultralytics自身のexport(format="onnx")を使う。

Usage:
    python3 export_cone_onnx.py \
        --weights /aiformula_machine/models/cone.pt \
        --output /aiformula_machine/web_simulator/models/cone.onnx
"""
import argparse
import shutil

from ultralytics import YOLO

DEFAULT_INPUT_SIZE = 640


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", default="/aiformula_machine/models/cone.pt")
    parser.add_argument("--output", default="/aiformula_machine/web_simulator/models/cone.onnx")
    parser.add_argument("--size", type=int, default=DEFAULT_INPUT_SIZE, help="Square input side length")
    parsed = parser.parse_args(args=args)

    model = YOLO(parsed.weights)
    exported_path = model.export(format="onnx", imgsz=parsed.size, opset=12, simplify=True)
    shutil.move(str(exported_path), parsed.output)
    print(f"[export_cone_onnx] Wrote ONNX graph: {parsed.output} (input: 1x3x{parsed.size}x{parsed.size})")


if __name__ == "__main__":
    main()
