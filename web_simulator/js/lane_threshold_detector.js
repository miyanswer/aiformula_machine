// Classical image-processing white-line detector: replaces the earlier
// YOLOP-ONNX inference path with a simple luminance + low-saturation
// threshold, run directly on the onboard-camera capture canvas. No model
// weights, no async model loading, no WASM runtime -- just a per-pixel scan,
// synchronous and fast enough to run every frame.
//
// A pixel counts as "white line" when it is both bright (luma above
// LUMA_THRESHOLD) and close to gray/white (low max-min channel spread, below
// MAX_CHROMA) -- this picks out the white lane markings on the course's gray
// road surface while rejecting the green off-road texture (which is bright
// in G but has a large channel spread) and the darker road itself.
//
// Downstream (js/oit_lane_pipeline.js's BevTransformer / BevLaneExtractor /
// stepPurePursuitControl) is unchanged: it only ever consumed a binary
// mask + width/height, regardless of how that mask was produced.

const TOP_CUT_RATIO = 0.45; // same ROI as before: ignore rows above the horizon-ish cut
const LUMA_THRESHOLD = 175; // 0-255; raise to require brighter/whiter pixels
const MAX_CHROMA = 40; // max(r,g,b) - min(r,g,b) must stay below this to count as "white/gray"

export class ThresholdLaneDetector {
  constructor() {
    this._canvas = document.createElement('canvas');
    this._ctx = this._canvas.getContext('2d', { willReadFrequently: true });
  }

  /**
   * @param {HTMLCanvasElement|OffscreenCanvas} sourceCanvas onboard camera capture (RGB)
   * @returns {Promise<{mask: Uint8Array, width: number, height: number}>}
   *   mask is 0/1, row-major, exactly `width`x`height` (= sourceCanvas size).
   */
  async infer(sourceCanvas) {
    const width = sourceCanvas.width;
    const height = sourceCanvas.height;
    this._canvas.width = width;
    this._canvas.height = height;
    this._ctx.drawImage(sourceCanvas, 0, 0, width, height);
    const { data } = this._ctx.getImageData(0, 0, width, height);

    const cutY = Math.round(height * TOP_CUT_RATIO);
    const mask = new Uint8Array(width * height);
    for (let y = cutY; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const idx = (y * width + x) * 4;
        const r = data[idx];
        const g = data[idx + 1];
        const b = data[idx + 2];
        const luma = (r + g + b) / 3;
        const maxc = Math.max(r, g, b);
        const minc = Math.min(r, g, b);
        if (luma > LUMA_THRESHOLD && maxc - minc < MAX_CHROMA) {
          mask[y * width + x] = 1;
        }
      }
    }
    return { mask, width, height };
  }
}
