// In-browser port of oit_navigation's lane_detector YOLOP backend
// (src/oit_navigation/oit_navigation/yolop_lane_backend.py) running the
// fine-tuned white-line segmentation model (models/honda_shihou_finetuned_best.pth,
// exported by src/oit_navigation/oit_navigation/export_onnx_web.py) through
// onnxruntime-web. This is the "YOLOP (ブラウザ)" detector mode -- the same
// model the real vehicle uses; the mask it returns is turned into per-line
// point sequences by extractMaskLines() (js/lane_navigator.js, port of
// lane_nav/mask_lines.py).
//
// Preprocessing: "crop_bottom" ROI (yolop_lane_backend.py's roi_mode option)
// -- drop the top TOP_CUT_RATIO fraction of the onboard-camera capture
// (sky/irrelevant background), then resize that remaining bottom crop
// directly to a 640x640 square for the model (no letterbox padding bars;
// the model's input is a plain stretched resize of the crop, matching how
// export_onnx_web.py fixes the ONNX graph's input shape at exactly 640x640).
//
// NOTE on channel order: yolop_lane_backend.py never converts its cv2/BGR
// frame to RGB before normalizing -- it feeds the raw BGR array straight
// into `transforms.Normalize(mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225])`
// (an ImageNet mean/std nominally defined for RGB). This port reproduces
// that same arrangement (channel 0 = Blue, channel 1 = Green, channel 2 =
// Red) since the fine-tuned checkpoint was trained through that same
// pipeline.

const MODEL_INPUT_SIZE = 640;
const TOP_CUT_RATIO = 0.45; // matches navigation_params.yaml top_cut_ratio (roi_mode: crop_bottom)
const NORM_MEAN = [0.485, 0.456, 0.406]; // applied to [B, G, R] in that order -- see module docstring
const NORM_STD = [0.229, 0.224, 0.225];

export class ModelLaneDetector {
  constructor() {
    this.session = null;
    this._cropCanvas = document.createElement('canvas');
    this._cropCanvas.width = MODEL_INPUT_SIZE;
    this._cropCanvas.height = MODEL_INPUT_SIZE;
    this._cropCtx = this._cropCanvas.getContext('2d', { willReadFrequently: true });
  }

  async load(onnxUrl, wasmDir) {
    // eslint-disable-next-line no-undef -- `ort` is the global UMD export of
    // vendor/onnxruntime-web/ort.webgpu.min.js, loaded via a plain <script>
    // tag in index.html before this module runs (same pattern as ROSLIB).
    // This bundle (unlike the plain ort.wasm.min.js one previously vendored
    // here) includes the 'webgpu' backend, so inference actually runs on the
    // GPU instead of silently falling back to single-threaded CPU WASM --
    // see the README's "実装上の注意" section for the jank this caused.
    //
    // Must be an absolute URL, not a bare relative path like "vendor/..." --
    // onnxruntime-web resolves its wasm/mjs glue files via `import()` under
    // the hood, and a specifier without a "./"/"../" prefix or scheme is an
    // invalid *bare* module specifier, which throws
    // "Failed to resolve module specifier" and takes down both the 'webgpu'
    // and 'wasm' backends (session creation throws before either EP loads).
    ort.env.wasm.wasmPaths = new URL(wasmDir, document.baseURI).href;
    // onnxruntime-web >=1.19 only ships pthread (SharedArrayBuffer) wasm
    // binaries -- the non-threaded build vendored here previously was
    // dropped upstream -- so both the 'webgpu' and 'wasm' backends now
    // require the page to be cross-origin isolated regardless of
    // ort.env.wasm.numThreads. See web_simulator/serve.py, which sends the
    // required COOP/COEP headers (plain `python3 -m http.server` does not).

    try {
      this.session = await ort.InferenceSession.create(onnxUrl, {
        executionProviders: ['webgpu'],
        graphOptimizationLevel: 'all',
      });
    } catch (err) {
      // No navigator.gpu (older browser, a WebGPU-less Electron/Chromium
      // build, or the page isn't cross-origin isolated) -- fall back to the
      // CPU WASM path so the model mode still works, just without the GPU
      // speedup.
      console.warn('WebGPU EP unavailable, falling back to CPU WASM inference', err);
      this.session = await ort.InferenceSession.create(onnxUrl, {
        executionProviders: ['wasm'],
        graphOptimizationLevel: 'all',
      });
    }
  }

  /**
   * @param {HTMLCanvasElement|OffscreenCanvas} sourceCanvas onboard camera capture (RGB)
   * @returns {Promise<{mask: Uint8Array, width: number, height: number}>}
   *   mask is 0/1, row-major, exactly `width`x`height` (= sourceCanvas size).
   */
  async infer(sourceCanvas) {
    if (!this.session) throw new Error('ModelLaneDetector.load() must complete before infer()');
    const width = sourceCanvas.width;
    const height = sourceCanvas.height;
    const cutY = Math.round(height * TOP_CUT_RATIO);
    const cropHeight = height - cutY;

    // crop_bottom + resize-to-square in one draw call (crops rows
    // [cutY, height) across the full width, then stretches to 640x640).
    this._cropCtx.drawImage(sourceCanvas, 0, cutY, width, cropHeight, 0, 0, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE);
    const cropImage = this._cropCtx.getImageData(0, 0, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE);

    const inputData = buildNormalizedInput(cropImage);
    const inputTensor = new ort.Tensor('float32', inputData, [1, 3, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE]);

    const results = await this.session.run({ input: inputTensor });
    const llSeg = results.ll_seg; // [1, 2, 640, 640], post-sigmoid (see export_onnx_web.py)

    return decodeLaneMask(llSeg.data, width, height, cutY, cropHeight);
  }
}

// Builds the normalized CHW input tensor (channel order [B, G, R] -- see
// module docstring) from the 640x640 cropped+resized RGBA ImageData.
function buildNormalizedInput(image) {
  const { data } = image;
  const planeSize = MODEL_INPUT_SIZE * MODEL_INPUT_SIZE;
  const out = new Float32Array(3 * planeSize);
  for (let i = 0; i < planeSize; i++) {
    const idx = i * 4;
    const r = data[idx];
    const g = data[idx + 1];
    const b = data[idx + 2];
    out[i] = (b / 255 - NORM_MEAN[0]) / NORM_STD[0];
    out[planeSize + i] = (g / 255 - NORM_MEAN[1]) / NORM_STD[1];
    out[2 * planeSize + i] = (r / 255 - NORM_MEAN[2]) / NORM_STD[2];
  }
  return out;
}

// Argmaxes the 640x640 model output, then maps it back down onto the
// original (width x height) camera frame: since the crop kept the full
// width (only rows were cropped), columns map 1:1 and only rows need
// nearest-neighbor resampling from the 640-tall model output back to
// cropHeight rows, placed at [cutY, height) in the full-size mask (rows
// above cutY -- the cropped-away sky -- stay 0, mirroring roi_mode
// "crop_bottom"'s zero-fill in yolop_lane_backend.py).
function decodeLaneMask(llSegData, width, height, cutY, cropHeight) {
  const mask = new Uint8Array(width * height);
  const planeSize = MODEL_INPUT_SIZE * MODEL_INPUT_SIZE;

  for (let y = 0; y < cropHeight; y++) {
    const modelRow = Math.min(MODEL_INPUT_SIZE - 1, Math.floor((y * MODEL_INPUT_SIZE) / cropHeight));
    const rowBase = modelRow * MODEL_INPUT_SIZE;
    const outRowBase = (y + cutY) * width;
    for (let x = 0; x < width; x++) {
      const idx = rowBase + x;
      const class0 = llSegData[idx];
      const class1 = llSegData[planeSize + idx];
      mask[outRowBase + x] = class1 > class0 ? 1 : 0;
    }
  }
  return { mask, width, height };
}
