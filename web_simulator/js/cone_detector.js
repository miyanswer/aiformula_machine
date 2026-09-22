// cone.onnx (Ultralytics YOLO検出, export_cone_onnx.pyで生成) を
// onnxruntime-webでブラウザ推論する。js/lane_model_detector.jsと同じ
// WebGPU->WASMフォールバック構成。コーンは接地物なので、信号機の
// (traffic_light_distance_node.pyのような)占有率からの距離逆算は不要 --
// バウンディングボックス下辺中央を lane_navigator.js の projectToGround()
// (白線検知と同じカメラモデル)にそのまま渡せば車体フレームの地面座標が
// 直接求まる。

import { projectToGround, DEFAULT_CAMERA } from './lane_navigator.js';

const MODEL_INPUT_SIZE = 640;
const CONF_THRESHOLD = 0.4;
const IOU_THRESHOLD = 0.45;
const VALID_X_MIN = 0.3, VALID_X_MAX = 8.0, VALID_Y_ABS_MAX = 3.0;

function iou(a, b) {
  const x1 = Math.max(a.x1, b.x1), y1 = Math.max(a.y1, b.y1);
  const x2 = Math.min(a.x2, b.x2), y2 = Math.min(a.y2, b.y2);
  const inter = Math.max(0, x2 - x1) * Math.max(0, y2 - y1);
  const areaA = (a.x2 - a.x1) * (a.y2 - a.y1);
  const areaB = (b.x2 - b.x1) * (b.y2 - b.y1);
  return inter / Math.max(areaA + areaB - inter, 1e-9);
}

/**
 * 単純なgreedy NMS。
 * @param {Array<{x1,y1,x2,y2,conf,cls}>} boxes
 * @returns {Array<{x1,y1,x2,y2,conf,cls}>}
 */
export function nonMaxSuppression(boxes, iouThreshold = IOU_THRESHOLD) {
  const sorted = [...boxes].sort((a, b) => b.conf - a.conf);
  const kept = [];
  for (const b of sorted) {
    if (kept.every((k) => iou(k, b) < iouThreshold)) kept.push(b);
  }
  return kept;
}

/**
 * Ultralytics YOLO ONNXの標準出力 [1, 4+numClasses, numBoxes] をデコードする。
 * @param {Float32Array} data
 * @param {number} numBoxes (例: 8400)
 * @param {number} numClasses
 * @param {number} confThreshold
 * @returns {Array<{x1,y1,x2,y2,conf,cls}>} 640x640モデル座標系のボックス
 */
export function decodeYoloOutput(data, numBoxes, numClasses, confThreshold = CONF_THRESHOLD) {
  const boxes = [];
  for (let i = 0; i < numBoxes; i++) {
    let bestCls = 0, bestConf = 0;
    for (let c = 0; c < numClasses; c++) {
      const conf = data[(4 + c) * numBoxes + i];
      if (conf > bestConf) { bestConf = conf; bestCls = c; }
    }
    if (bestConf < confThreshold) continue;
    const cx = data[0 * numBoxes + i], cy = data[1 * numBoxes + i];
    const w = data[2 * numBoxes + i], h = data[3 * numBoxes + i];
    boxes.push({ x1: cx - w / 2, y1: cy - h / 2, x2: cx + w / 2, y2: cy + h / 2, conf: bestConf, cls: bestCls });
  }
  return nonMaxSuppression(boxes);
}

/**
 * モデル座標(640x640, レターボックス)のボックスを、元画像ピクセル座標に戻す。
 */
export function unletterbox(box, srcWidth, srcHeight, modelSize = MODEL_INPUT_SIZE) {
  const scale = Math.min(modelSize / srcWidth, modelSize / srcHeight);
  const padX = (modelSize - srcWidth * scale) / 2;
  const padY = (modelSize - srcHeight * scale) / 2;
  return {
    x1: (box.x1 - padX) / scale, y1: (box.y1 - padY) / scale,
    x2: (box.x2 - padX) / scale, y2: (box.y2 - padY) / scale,
    conf: box.conf, cls: box.cls,
  };
}

/**
 * バウンディングボックス下辺中央を地面に投影し、有効範囲でフィルタする。
 * @param {Array<{x1,y1,x2,y2,conf}>} boxes 元画像ピクセル座標
 * @param {number} width 元画像幅
 * @param {number} height 元画像高さ
 * @returns {Array<{x:number, y:number, conf:number}>}
 */
export function groundPositionsFromBoxes(boxes, width, height) {
  const out = [];
  for (const b of boxes) {
    const u = [(b.x1 + b.x2) / 2];
    const v = [b.y2];
    const g = projectToGround(DEFAULT_CAMERA, u, v, width, height);
    if (g.x.length === 0) continue;
    const x = g.x[0], y = g.y[0];
    if (x < VALID_X_MIN || x > VALID_X_MAX || Math.abs(y) > VALID_Y_ABS_MAX) continue;
    out.push({ x, y, conf: b.conf });
  }
  return out;
}

export class ConeDetector {
  constructor() {
    this.session = null;
    this._letterboxCanvas = document.createElement('canvas');
    this._letterboxCanvas.width = MODEL_INPUT_SIZE;
    this._letterboxCanvas.height = MODEL_INPUT_SIZE;
    this._letterboxCtx = this._letterboxCanvas.getContext('2d', { willReadFrequently: true });
  }

  async load(onnxUrl, wasmDir) {
    // eslint-disable-next-line no-undef -- js/lane_model_detector.jsと同じ、
    // index.htmlの<script>タグで読み込まれるortグローバル。
    ort.env.wasm.wasmPaths = new URL(wasmDir, document.baseURI).href;
    try {
      this.session = await ort.InferenceSession.create(onnxUrl, {
        executionProviders: ['webgpu'],
        graphOptimizationLevel: 'all',
      });
    } catch (err) {
      console.warn('ConeDetector: WebGPU EP unavailable, falling back to CPU WASM', err);
      this.session = await ort.InferenceSession.create(onnxUrl, {
        executionProviders: ['wasm'],
        graphOptimizationLevel: 'all',
      });
    }
  }

  /**
   * @param {HTMLCanvasElement|OffscreenCanvas} sourceCanvas
   * @returns {Promise<Array<{x:number, y:number, conf:number}>>} 車体フレーム地面座標、有効範囲フィルタ済み
   */
  async infer(sourceCanvas) {
    if (!this.session) throw new Error('ConeDetector.load() must complete before infer()');
    const width = sourceCanvas.width, height = sourceCanvas.height;
    const scale = Math.min(MODEL_INPUT_SIZE / width, MODEL_INPUT_SIZE / height);
    const drawW = width * scale, drawH = height * scale;
    const padX = (MODEL_INPUT_SIZE - drawW) / 2, padY = (MODEL_INPUT_SIZE - drawH) / 2;

    this._letterboxCtx.fillStyle = '#727272';
    this._letterboxCtx.fillRect(0, 0, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE);
    this._letterboxCtx.drawImage(sourceCanvas, 0, 0, width, height, padX, padY, drawW, drawH);
    const image = this._letterboxCtx.getImageData(0, 0, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE);

    const planeSize = MODEL_INPUT_SIZE * MODEL_INPUT_SIZE;
    const input = new Float32Array(3 * planeSize);
    for (let i = 0; i < planeSize; i++) {
      const idx = i * 4;
      input[i] = image.data[idx] / 255;
      input[planeSize + i] = image.data[idx + 1] / 255;
      input[2 * planeSize + i] = image.data[idx + 2] / 255;
    }
    const inputTensor = new ort.Tensor('float32', input, [1, 3, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE]);
    const results = await this.session.run({ images: inputTensor });
    const outputKey = Object.keys(results)[0];
    const output = results[outputKey];
    const numClasses = output.dims[1] - 4;
    const numBoxes = output.dims[2];

    const modelBoxes = decodeYoloOutput(output.data, numBoxes, numClasses);
    const pixelBoxes = modelBoxes.map((b) => unletterbox(b, width, height));
    return groundPositionsFromBoxes(pixelBoxes, width, height);
  }
}
