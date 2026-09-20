"""
export_ufld_onnx.py - UFLD v1 の重み (.pth) を ONNX に書き出す.

用途:
    - Web シミュレータ (web_simulator/js/ufld_lane_detector.js) のブラウザ内推論
      (onnxruntime-web) 用のモデル web_simulator/models/ufld.onnx を作る.
    - Jetson で TensorRT エンジンを作る際の中間ファイル (trtexec --onnx=...).

出力テンソル:
    input : float32 [1, 3, 288, 800]  (RGB, ImageNet mean/std 正規化済み)
    output: float32 [1, griding_num+1, cls_num_per_lane, num_lanes] (生 logits)
    デコードは oit_navigation/ufld/model.py の decode_lanes() と同じ処理を JS 側で行う.

使い方:
    ros2 run oit_navigation export_ufld_onnx
    ros2 run oit_navigation export_ufld_onnx --weight models/ufld_honda_finetuned_best.pth \
        --output web_simulator/models/ufld.onnx

注意: ResNet18 版で約 190MB になる (全結合層 2048x22624 が大半) ため git には含めない
(.gitignore 済み). 各自この スクリプトで生成すること.
"""

import argparse
import json
import os

from common_python.workspace_paths import default_workspace_asset, resolve_workspace_asset


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weight", default=default_workspace_asset("models", "ufld_honda_finetuned_best.pth"))
    parser.add_argument("--output", default=default_workspace_asset("web_simulator", "models", "ufld.onnx"))
    parser.add_argument("--opset", type=int, default=17)
    args = parser.parse_args(argv)

    import torch
    from oit_navigation.ufld.model import UFLDLaneModel

    weight = resolve_workspace_asset(args.weight)
    model = UFLDLaneModel(weight, device="cpu")
    cfg = model.cfg
    dummy = torch.zeros(1, 3, cfg.input_h, cfg.input_w)

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    torch.onnx.export(
        model.net, dummy, args.output,
        input_names=["input"], output_names=["output"],
        opset_version=args.opset, do_constant_folding=True,
    )

    # JS 側がデコードに使う設定を横に置く (row anchor / griding_num 等)
    meta_path = os.path.splitext(args.output)[0] + ".json"
    with open(meta_path, "w") as f:
        json.dump({
            "griding_num": cfg.griding_num,
            "cls_num_per_lane": cfg.cls_num_per_lane,
            "num_lanes": cfg.num_lanes,
            "row_anchor": cfg.row_anchor,
            "input_h": cfg.input_h,
            "input_w": cfg.input_w,
        }, f, indent=1)
    print(f"[export_ufld_onnx] {weight} -> {args.output} (+ {meta_path})")


if __name__ == "__main__":
    main()
