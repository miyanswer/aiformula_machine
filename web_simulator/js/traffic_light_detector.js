// traffic_light.onnx (models/traffic_light.pt を export_cone_onnx.py で変換した Ultralytics YOLO 検出モデル、
// クラス 0 = traffic_light_red / 1 = traffic_light_green) を onnxruntime-web でブラウザ推論し、
// 実機の traffic_light_distance_node.py と同じ「バウンディングボックスの縦の画面占有率」から距離を逆算する:
//   occupancy = bbox_h / image_h,  distance = coeff / occupancy,  coeff = real_height * fy / reference_height
// MyLaps パネルは実機と同じ 32cm 角 (models/MyLaps.obj の Panel_Body)。
// 焦点距離は実機 (traffic_light_params.yaml の focal_length_y = 実測校正値) と同じく実測で校正した値:
// シミュレータのカメラの幾何的な fy は 763.17px @1080 (縦画角 70.6deg) だが、YOLO のボックスは小さい物体ほど
// 実物より数 px 大きく出る (640 入力に縮小するため。実機も同じ) ので、そのままだと距離を 1〜2m 短く見積もる。
// 3〜18m に置いた赤/緑パネル 120 枚で合わせ、停止帯 4〜8m で誤差平均 ±0.1m になる 900px を使う
// (12m より遠いとボックスが 8〜10px で頭打ちになり短めに出る = 早めに減速するだけで安全側)。
// 生成: python3 src/oit_navigation/oit_navigation/export_cone_onnx.py \
//         --weights models/traffic_light.pt --output web_simulator/models/traffic_light.onnx

import { decodeYoloOutput, unletterbox } from './cone_detector.js';

export const SIM_TRAFFIC_LIGHT_FOCAL_LENGTH_Y = 900.0; // [px @1080]

const MODEL_INPUT_SIZE = 640;

// traffic_light_params.yaml / traffic_light_distance.py と同じ既定値 (焦点距離だけシミュレータ用に校正した値)
export const TRAFFIC_LIGHT_DETECTOR_PARAMS = {
  confThreshold: 0.35,
  realHeightM: 0.32,
  focalLengthY: SIM_TRAFFIC_LIGHT_FOCAL_LENGTH_Y,
  referenceImageHeight: 1080,
  minOccupancy: 1e-4,
  maxValidDistance: 50.0,
  smoothingAlpha: 0.7,
};

const CLASS_NAMES = ['traffic_light_red', 'traffic_light_green'];

/** traffic_light_distance.py の TrafficLightDistanceEstimator と同じ計算。 */
export class TrafficLightDistanceEstimator {
  constructor(p = TRAFFIC_LIGHT_DETECTOR_PARAMS) {
    this.p = p;
    this.distanceCoeff = (p.realHeightM * p.focalLengthY) / p.referenceImageHeight;
    this._cache = new Map();
  }

  estimate(bboxHeightPx, imageHeightPx, trackId = null) {
    if (imageHeightPx <= 0 || bboxHeightPx <= 0) return null;
    const occupancy = bboxHeightPx / imageHeightPx;
    if (occupancy < this.p.minOccupancy) return null;
    const raw = this.distanceCoeff / occupancy;
    if (raw <= 0 || raw > this.p.maxValidDistance) return null;
    const a = this.p.smoothingAlpha;
    if (trackId !== null && a > 0 && a < 1) {
      const prev = this._cache.get(trackId);
      const smoothed = prev === undefined ? raw : a * raw + (1 - a) * prev;
      this._cache.set(trackId, smoothed);
      return smoothed;
    }
    return raw;
  }
}

export class TrafficLightDetector {
  constructor(params = TRAFFIC_LIGHT_DETECTOR_PARAMS) {
    this.p = params;
    this.session = null;
    this.estimator = new TrafficLightDistanceEstimator(params);
    this.lastInferMs = 0;
    this._letterboxCanvas = document.createElement('canvas');
    this._letterboxCanvas.width = MODEL_INPUT_SIZE;
    this._letterboxCanvas.height = MODEL_INPUT_SIZE;
    this._letterboxCtx = this._letterboxCanvas.getContext('2d', { willReadFrequently: true });
  }

  async load(onnxUrl, wasmDir) {
    // eslint-disable-next-line no-undef -- index.html の <script> で読み込む ort グローバル
    ort.env.wasm.wasmPaths = new URL(wasmDir, document.baseURI).href;
    try {
      this.session = await ort.InferenceSession.create(onnxUrl, { executionProviders: ['webgpu'], graphOptimizationLevel: 'all' });
    } catch (err) {
      console.warn('TrafficLightDetector: WebGPU EP unavailable, falling back to CPU WASM', err);
      this.session = await ort.InferenceSession.create(onnxUrl, { executionProviders: ['wasm'], graphOptimizationLevel: 'all' });
    }
  }

  /**
   * @param {HTMLCanvasElement} sourceCanvas オンボードカメラ画像
   * @returns {Promise<{red:number|null, green:number|null, detections:Array}>} 最も近い赤/青までの距離 [m]
   */
  async infer(sourceCanvas) {
    if (!this.session) throw new Error('TrafficLightDetector.load() must complete before infer()');
    const t0 = performance.now();
    const width = sourceCanvas.width, height = sourceCanvas.height;
    const scale = Math.min(MODEL_INPUT_SIZE / width, MODEL_INPUT_SIZE / height);
    const drawW = width * scale, drawH = height * scale;
    const padX = (MODEL_INPUT_SIZE - drawW) / 2, padY = (MODEL_INPUT_SIZE - drawH) / 2;
    const ctx = this._letterboxCtx;
    ctx.fillStyle = '#727272';
    ctx.fillRect(0, 0, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE);
    ctx.drawImage(sourceCanvas, 0, 0, width, height, padX, padY, drawW, drawH);
    const image = ctx.getImageData(0, 0, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE);
    const plane = MODEL_INPUT_SIZE * MODEL_INPUT_SIZE;
    const input = new Float32Array(3 * plane);
    for (let i = 0; i < plane; i++) {
      input[i] = image.data[i * 4] / 255;
      input[plane + i] = image.data[i * 4 + 1] / 255;
      input[2 * plane + i] = image.data[i * 4 + 2] / 255;
    }
    // eslint-disable-next-line no-undef
    const results = await this.session.run({ images: new ort.Tensor('float32', input, [1, 3, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE]) });
    const output = results[Object.keys(results)[0]];
    const boxes = decodeYoloOutput(output.data, output.dims[2], output.dims[1] - 4, this.p.confThreshold)
      .map((b) => unletterbox(b, width, height));

    let red = null, green = null;
    const detections = boxes.map((b, i) => {
      const bboxH = Math.abs(b.y2 - b.y1);
      const distance = this.estimator.estimate(bboxH, height, i);
      const className = CLASS_NAMES[b.cls] ?? `class_${b.cls}`;
      if (distance !== null) {
        if (b.cls === 0 && (red === null || distance < red)) red = distance;
        if (b.cls === 1 && (green === null || distance < green)) green = distance;
      }
      return { className, cls: b.cls, conf: b.conf, bbox: [b.x1, b.y1, b.x2, b.y2], bboxH, distance };
    });
    this.lastInferMs = performance.now() - t0;
    return { red, green, detections };
  }
}
