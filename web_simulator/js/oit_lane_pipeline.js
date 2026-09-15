// In-browser port of oit_navigation's BEV transform + sliding-window lane
// extraction + Pure Pursuit control law, so the web simulator can run the
// same lane-tracking pipeline the real vehicle runs (see
// src/oit_navigation/oit_navigation/utils/bev_transformer.py,
// utils/bev_lane_extractor.py, and bev_pure_pursuit_node.py's
// _run_control_step, which this file ports section-by-section, in that
// order). Only what bev_pure_pursuit_node.py actually calls at runtime is
// ported -- e.g. BEVTransformer's coordinate-conversion helpers and the
// (imported-but-unused) AdaptivePurePursuit class are not reproduced here.

// ---------------------------------------------------------------------------
// Small linear-algebra helpers (Gaussian elimination with partial pivoting),
// shared by the homography solve and the regularized polynomial fit below.
// ---------------------------------------------------------------------------
function solveLinearSystem(A, b) {
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let col = 0; col < n; col++) {
    let pivot = col;
    for (let r = col + 1; r < n; r++) {
      if (Math.abs(M[r][col]) > Math.abs(M[pivot][col])) pivot = r;
    }
    [M[col], M[pivot]] = [M[pivot], M[col]];
    const pivotVal = M[col][col];
    if (Math.abs(pivotVal) < 1e-12) return null;
    for (let c = col; c <= n; c++) M[col][c] /= pivotVal;
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const factor = M[r][col];
      if (factor === 0) continue;
      for (let c = col; c <= n; c++) M[r][c] -= factor * M[col][c];
    }
  }
  return M.map((row) => row[n]);
}

// Direct Linear Transform for a 4-point planar homography mapping
// srcPts[i] -> dstPts[i]. Returns a 3x3 matrix as a flat 9-element array.
function computeHomography(srcPts, dstPts) {
  const A = [];
  const b = [];
  for (let i = 0; i < 4; i++) {
    const [x, y] = srcPts[i];
    const [u, v] = dstPts[i];
    A.push([x, y, 1, 0, 0, 0, -x * u, -y * u]);
    b.push(u);
    A.push([0, 0, 0, x, y, 1, -x * v, -y * v]);
    b.push(v);
  }
  const h = solveLinearSystem(A, b);
  if (!h) return null;
  return [h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1];
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

// ---------------------------------------------------------------------------
// BEVTransformer port (utils/bev_transformer.py). Only warp_to_bev is used
// by bev_pure_pursuit_node.py at runtime, so that's all this reproduces --
// built as the dst->src homography directly (equivalent to Python's M_inv),
// since that's the only direction warping actually needs.
// ---------------------------------------------------------------------------
export class BevTransformer {
  constructor({
    camHeight = 0.56, // Camera ground clearance [m]
    camFocal = 1050.0, // 1080p-equivalent focal length [px]
    distMin = 1.15, // Forward distance min [m]
    distMax = 10.0, // Forward distance max [m]
    lateralMax = 5.0, // Lateral half-width [m]
    bevW = 640,
    bevH = 640,
  } = {}) {
    this.camHeight = camHeight;
    this.camFocal = camFocal;
    this.distMin = distMin;
    this.distMax = distMax;
    this.lateralMax = lateralMax;
    this.bevW = bevW;
    this.bevH = bevH;
    this.vNear = (1.0 - distMin / distMax) * bevH;
    this.srcW = 0;
    this.srcH = 0;
    this._invHomography = null;
  }

  _buildInverseHomography(srcW, srcH) {
    const cx = srcW / 2.0;
    const cy = srcH / 2.0;
    const fPx = this.camFocal * (srcW / 1920.0);

    const srcPts = [
      [cx - fPx * (this.lateralMax / this.distMin), cy + fPx * (this.camHeight / this.distMin)], // Near-Left
      [cx + fPx * (this.lateralMax / this.distMin), cy + fPx * (this.camHeight / this.distMin)], // Near-Right
      [cx + fPx * (this.lateralMax / this.distMax), cy + fPx * (this.camHeight / this.distMax)], // Far-Right
      [cx - fPx * (this.lateralMax / this.distMax), cy + fPx * (this.camHeight / this.distMax)], // Far-Left
    ];
    const dstPts = [
      [0, this.vNear],
      [this.bevW, this.vNear],
      [this.bevW, 0],
      [0, 0],
    ];

    this._invHomography = computeHomography(dstPts, srcPts); // dst -> src (what warping needs)
    this.srcW = srcW;
    this.srcH = srcH;
  }

  /** Nearest-neighbor warp of a binary mask into BEV space (is_binary=True path only). */
  warpToBev(maskUint8, srcW, srcH) {
    if (srcW !== this.srcW || srcH !== this.srcH || !this._invHomography) {
      this._buildInverseHomography(srcW, srcH);
    }
    const h = this._invHomography;
    const out = new Uint8Array(this.bevW * this.bevH);
    for (let v = 0; v < this.bevH; v++) {
      for (let u = 0; u < this.bevW; u++) {
        const w = h[6] * u + h[7] * v + h[8];
        const sx = Math.round((h[0] * u + h[1] * v + h[2]) / w);
        const sy = Math.round((h[3] * u + h[4] * v + h[5]) / w);
        if (sx >= 0 && sx < srcW && sy >= 0 && sy < srcH) {
          out[v * this.bevW + u] = maskUint8[sy * srcW + sx];
        }
      }
    }
    return out;
  }
}

// ---------------------------------------------------------------------------
// BEVLaneExtractor port (utils/bev_lane_extractor.py).
// ---------------------------------------------------------------------------
function reflect101(i, n) {
  if (n === 1) return 0;
  const period = 2 * (n - 1);
  i = ((i % period) + period) % period;
  return i >= n ? period - i : i;
}

function gaussianKernel1D(ksize, sigma) {
  const half = (ksize - 1) / 2;
  const kernel = new Array(ksize);
  let sum = 0;
  for (let i = 0; i < ksize; i++) {
    const x = i - half;
    kernel[i] = Math.exp(-(x * x) / (2 * sigma * sigma));
    sum += kernel[i];
  }
  return kernel.map((v) => v / sum);
}

const HIST_KERNEL = gaussianKernel1D(15, 3.0);

function gaussianBlur1D(arr, kernel) {
  const n = arr.length;
  const half = (kernel.length - 1) / 2;
  const out = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    let acc = 0;
    for (let k = 0; k < kernel.length; k++) {
      acc += arr[reflect101(i + k - half, n)] * kernel[k];
    }
    out[i] = acc;
  }
  return out;
}

function linspace(start, stop, num) {
  const out = new Float64Array(num);
  const step = (stop - start) / (num - 1);
  for (let i = 0; i < num; i++) out[i] = start + step * i;
  return out;
}

function argminAbs(arr, target) {
  let bestIdx = 0;
  let bestVal = Infinity;
  for (let i = 0; i < arr.length; i++) {
    const d = Math.abs(arr[i] - target);
    if (d < bestVal) {
      bestVal = d;
      bestIdx = i;
    }
  }
  return bestIdx;
}

function polyval(coeffs, x) {
  // coeffs = [a, b, c] for a*x^2 + b*x + c (np.polyfit/np.polyval order)
  return coeffs[0] * x * x + coeffs[1] * x + coeffs[2];
}

export class BevLaneExtractor {
  constructor({
    bevW = 640,
    bevH = 640,
    laneWidthM = 3.5,
    distMin = 1.15,
    distMax = 10.0,
    lateralMax = 5.0,
    nWindows = 14,
    marginPx = 45,
    minRecenterPixels = 15,
    emaAlpha = 0.6,
  } = {}) {
    this.bevW = bevW;
    this.bevH = bevH;
    this.laneWidthM = laneWidthM;
    this.halfWidthM = laneWidthM / 2.0;
    this.distMin = distMin;
    this.distMax = distMax;
    this.lateralMax = lateralMax;
    this.nWindows = nWindows;
    this.marginPx = marginPx;
    this.minRecenterPixels = minRecenterPixels;
    this.emaAlpha = emaAlpha;
    this.vNear = (1.0 - distMin / distMax) * bevH;

    this.smoothedLeftCoeffs = null;
    this.smoothedRightCoeffs = null;
    this.smoothedCenterCoeffs = null;
  }

  _pxToMetric(uArr, vArr) {
    const pts = [];
    for (let i = 0; i < uArr.length; i++) {
      const x = this.distMax - (vArr[i] / this.vNear) * (this.distMax - this.distMin);
      const y = this.lateralMax - (uArr[i] / this.bevW) * (2.0 * this.lateralMax);
      if (x >= this.distMin && x <= this.distMax) pts.push([x, y]);
    }
    pts.sort((a, b) => a[0] - b[0]);
    return pts;
  }

  _metricToPx(x, y) {
    const u = Math.round(((this.lateralMax - y) / (2.0 * this.lateralMax)) * this.bevW);
    const v = Math.round(((this.distMax - x) / (this.distMax - this.distMin)) * this.vNear);
    return [u, v];
  }

  _findNearestPeak(subHist, isLeft, midU) {
    let maxVal = 0;
    for (let i = 0; i < subHist.length; i++) maxVal = Math.max(maxVal, subHist[i]);
    if (maxVal < 10) {
      return isLeft ? midU - Math.round(this.bevW * 0.22) : midU + Math.round(this.bevW * 0.22);
    }
    const threshold = maxVal * 0.35;
    const peaks = [];
    for (let i = 1; i < subHist.length - 1; i++) {
      if (subHist[i] > threshold && subHist[i] >= subHist[i - 1] && subHist[i] >= subHist[i + 1]) {
        peaks.push(i);
      }
    }
    if (peaks.length === 0) {
      let argMax = 0;
      for (let i = 1; i < subHist.length; i++) if (subHist[i] > subHist[argMax]) argMax = i;
      return isLeft ? argMax : argMax + midU;
    }
    return isLeft ? peaks[peaks.length - 1] : midU + peaks[0];
  }

  _fitRegularizedPoly(ptsMetric) {
    const n = ptsMetric.length;
    if (n < 4) return null;
    const xs = ptsMetric.map((p) => p[0]);
    const ys = ptsMetric.map((p) => p[1]);
    const xMin = Math.min(...xs);
    const xMax = Math.max(...xs);

    if (xMax - xMin < 1.0 || n < 8) {
      // Degree-1 fallback (np.polyfit(x, y, deg=1)) via normal equations.
      let sx = 0, sy = 0, sxx = 0, sxy = 0;
      for (let i = 0; i < n; i++) {
        sx += xs[i]; sy += ys[i]; sxx += xs[i] * xs[i]; sxy += xs[i] * ys[i];
      }
      const denom = n * sxx - sx * sx;
      if (Math.abs(denom) < 1e-9) return null;
      const bCoef = (n * sxy - sx * sy) / denom;
      const cCoef = (sy - bCoef * sx) / n;
      return [0.0, bCoef, cCoef];
    }

    // X^T X + reg, X^T y for columns [x^2, x, 1]
    const regLambda = 0.3;
    const reg = [regLambda, 0.01, 0.001];
    const XtX = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    const Xty = [0, 0, 0];
    for (let i = 0; i < n; i++) {
      const row = [xs[i] * xs[i], xs[i], 1];
      for (let r = 0; r < 3; r++) {
        Xty[r] += row[r] * ys[i];
        for (let c = 0; c < 3; c++) XtX[r][c] += row[r] * row[c];
      }
    }
    for (let i = 0; i < 3; i++) XtX[i][i] += reg[i];
    const coeffs = solveLinearSystem(XtX, Xty);
    if (!coeffs) return null;
    coeffs[0] = clamp(coeffs[0], -0.1, 0.1);
    return coeffs;
  }

  _fitAndScore(ptsMetric) {
    if (!ptsMetric || ptsMetric.length < 6) return { coeffs: null, quality: 0.0 };
    const coeffs = this._fitRegularizedPoly(ptsMetric);
    if (!coeffs) return { coeffs: null, quality: 0.0 };

    const xs = ptsMetric.map((p) => p[0]);
    const ys = ptsMetric.map((p) => p[1]);
    const span = Math.max(...xs) - Math.min(...xs);
    const spanScore = clamp(span / 5.0, 0.0, 1.0);
    const densityScore = clamp(xs.length / 60.0, 0.0, 1.0);

    let mse = 0;
    for (let i = 0; i < xs.length; i++) {
      const e = ys[i] - polyval(coeffs, xs[i]);
      mse += e * e;
    }
    mse /= xs.length;
    const smoothScore = clamp(1.0 / (1.0 + 10.0 * mse), 0.0, 1.0);

    const quality = 0.5 * spanScore + 0.3 * densityScore + 0.2 * smoothScore;
    return { coeffs, quality };
  }

  _extrapolatePreviousCenter(sampleX) {
    if (!this.smoothedCenterCoeffs) return null;
    return Array.from(sampleX, (x) => [x, polyval(this.smoothedCenterCoeffs, x)]);
  }

  /**
   * @param {Uint8Array} bevMask flattened bevW*bevH binary mask
   * @param {CanvasRenderingContext2D|null} ctx optional canvas to draw the
   *   annotated BEV visualization onto (mirrors annotated_bev in Python)
   * @returns {{targetCenter: Array<[number,number]>|null, leftPts: Array<[number,number]>|null, rightPts: Array<[number,number]>|null}}
   */
  extractLaneTrajectories(bevMask, ctx) {
    if (ctx) this._drawBaseMask(ctx, bevMask);

    const nonzero = [];
    for (let v = 0; v < this.bevH; v++) {
      for (let u = 0; u < this.bevW; u++) {
        if (bevMask[v * this.bevW + u] > 0) nonzero.push([u, v]);
      }
    }

    const sampleX = linspace(0.0, 7.5, 50);

    if (nonzero.length < 20) {
      // Matches Python: the sparse-mask early return skips the target-line
      // / ego-marker drawing below entirely (annotated_bev stays just the
      // raw mask).
      const targetCenter = this._extrapolatePreviousCenter(sampleX);
      return { targetCenter, leftPts: null, rightPts: null };
    }

    // 1. Base Anchor Peak Search
    const midU = Math.floor(this.bevW / 2);
    const bottomStart = Math.floor(this.vNear * 0.4);
    const bottomEnd = Math.floor(this.vNear);

    const histogram = new Float64Array(this.bevW);
    for (const [u, v] of nonzero) {
      if (v >= bottomStart && v < bottomEnd) histogram[u] += 1;
    }
    const smoothedHist = gaussianBlur1D(histogram, HIST_KERNEL);

    const leftBase = this._findNearestPeak(smoothedHist.slice(0, midU), true, midU);
    const rightBase = this._findNearestPeak(smoothedHist.slice(midU), false, midU);

    // 2. Inertial Sliding Window Tracing
    const windowHeight = Math.floor(this.vNear / this.nWindows);
    let leftCurrent = leftBase;
    let rightCurrent = rightBase;
    let leftDx = 0.0;
    let rightDx = 0.0;

    const leftIndsAll = [];
    const rightIndsAll = [];

    for (let w = 0; w < this.nWindows; w++) {
      const winYLow = Math.floor(this.vNear - (w + 1) * windowHeight);
      const winYHigh = Math.floor(this.vNear - w * windowHeight);
      const winXLeftLow = Math.floor(leftCurrent - this.marginPx);
      const winXLeftHigh = Math.floor(leftCurrent + this.marginPx);
      const winXRightLow = Math.floor(rightCurrent - this.marginPx);
      const winXRightHigh = Math.floor(rightCurrent + this.marginPx);

      if (ctx) {
        ctx.strokeStyle = 'rgb(0,60,100)'; // BGR (100,60,0) -> RGB
        ctx.strokeRect(winXLeftLow, winYLow, winXLeftHigh - winXLeftLow, winYHigh - winYLow);
        ctx.strokeStyle = 'rgb(110,80,0)'; // BGR (0,80,110) -> RGB
        ctx.strokeRect(winXRightLow, winYLow, winXRightHigh - winXRightLow, winYHigh - winYLow);
      }

      const goodLeft = [];
      const goodRight = [];
      for (const [u, v] of nonzero) {
        if (v >= winYLow && v < winYHigh) {
          if (u >= winXLeftLow && u < winXLeftHigh) goodLeft.push([u, v]);
          if (u >= winXRightLow && u < winXRightHigh) goodRight.push([u, v]);
        }
      }
      leftIndsAll.push(...goodLeft);
      rightIndsAll.push(...goodRight);

      if (goodLeft.length >= this.minRecenterPixels) {
        const newLeft = goodLeft.reduce((a, p) => a + p[0], 0) / goodLeft.length;
        leftDx = 0.7 * leftDx + 0.3 * (newLeft - leftCurrent);
        leftCurrent = newLeft;
      } else {
        leftCurrent += leftDx;
      }

      if (goodRight.length >= this.minRecenterPixels) {
        const newRight = goodRight.reduce((a, p) => a + p[0], 0) / goodRight.length;
        rightDx = 0.7 * rightDx + 0.3 * (newRight - rightCurrent);
        rightCurrent = newRight;
      } else {
        rightCurrent += rightDx;
      }
    }

    let leftPtsPx = null;
    let rightPtsPx = null;
    if (leftIndsAll.length >= 15) {
      leftPtsPx = leftIndsAll;
      if (ctx) {
        ctx.fillStyle = 'rgb(0,180,255)'; // BGR (255,180,0) -> RGB
        for (const [u, v] of leftIndsAll) ctx.fillRect(u, v, 1, 1);
      }
    }
    if (rightIndsAll.length >= 15) {
      rightPtsPx = rightIndsAll;
      if (ctx) {
        ctx.fillStyle = 'rgb(255,220,0)'; // BGR (0,220,255) -> RGB
        for (const [u, v] of rightIndsAll) ctx.fillRect(u, v, 1, 1);
      }
    }

    const leftPtsMetric = leftPtsPx ? this._pxToMetric(leftPtsPx.map((p) => p[0]), leftPtsPx.map((p) => p[1])) : null;
    const rightPtsMetric = rightPtsPx ? this._pxToMetric(rightPtsPx.map((p) => p[0]), rightPtsPx.map((p) => p[1])) : null;

    // 3. Fit polynomials & score quality
    const { coeffs: lCoeffs, quality: lQual } = this._fitAndScore(leftPtsMetric);
    const { coeffs: rCoeffs, quality: rQual } = this._fitAndScore(rightPtsMetric);

    if (lCoeffs) {
      this.smoothedLeftCoeffs = this.smoothedLeftCoeffs
        ? lCoeffs.map((v, i) => this.emaAlpha * v + (1 - this.emaAlpha) * this.smoothedLeftCoeffs[i])
        : lCoeffs;
    }
    if (rCoeffs) {
      this.smoothedRightCoeffs = this.smoothedRightCoeffs
        ? rCoeffs.map((v, i) => this.emaAlpha * v + (1 - this.emaAlpha) * this.smoothedRightCoeffs[i])
        : rCoeffs;
    }

    // 4. Anchor-First 3.5m Road Offset Strategy
    const hasLeft = this.smoothedLeftCoeffs !== null && lQual > 0.15;
    const hasRight = this.smoothedRightCoeffs !== null && rQual > 0.15;

    let targetCenter = null;
    if (hasLeft && hasRight) {
      if (lQual >= 1.4 * rQual) {
        targetCenter = Array.from(sampleX, (x) => [x, polyval(this.smoothedLeftCoeffs, x) - this.halfWidthM]);
      } else if (rQual >= 1.4 * lQual) {
        targetCenter = Array.from(sampleX, (x) => [x, polyval(this.smoothedRightCoeffs, x) + this.halfWidthM]);
      } else {
        const wTot = lQual + rQual;
        targetCenter = Array.from(sampleX, (x) => {
          const lY = polyval(this.smoothedLeftCoeffs, x) - this.halfWidthM;
          const rY = polyval(this.smoothedRightCoeffs, x) + this.halfWidthM;
          return [x, (lQual * lY + rQual * rY) / wTot];
        });
      }
    } else if (hasLeft) {
      targetCenter = Array.from(sampleX, (x) => [x, polyval(this.smoothedLeftCoeffs, x) - this.halfWidthM]);
    } else if (hasRight) {
      targetCenter = Array.from(sampleX, (x) => [x, polyval(this.smoothedRightCoeffs, x) + this.halfWidthM]);
    } else {
      targetCenter = this._extrapolatePreviousCenter(sampleX);
    }

    if (targetCenter) {
      const cFit = this._fitRegularizedPoly(targetCenter);
      if (cFit) {
        this.smoothedCenterCoeffs = this.smoothedCenterCoeffs
          ? cFit.map((v, i) => this.emaAlpha * v + (1 - this.emaAlpha) * this.smoothedCenterCoeffs[i])
          : cFit;
        targetCenter = Array.from(sampleX, (x) => [x, polyval(this.smoothedCenterCoeffs, x)]);
      }
    }

    // Note: the anchor-offset `targetCenter` line computed above is NOT
    // drawn here. The line actually drawn onto the BEV panel is the
    // origin-anchored Pure Pursuit trajectory (which passes through the
    // vehicle's own origin by construction) -- see drawTrajectory() below,
    // called from js/simulator.js once stepPurePursuitControl() has
    // computed that trajectory. Drawing `targetCenter` here instead would
    // show a line that doesn't necessarily touch the EGO marker (it's
    // purely a lane-geometry offset line, not anchored at the vehicle).

    return { targetCenter, leftPts: leftPtsMetric, rightPts: rightPtsMetric };
  }

  _drawBaseMask(ctx, bevMask) {
    const imgData = ctx.createImageData(this.bevW, this.bevH);
    for (let i = 0; i < bevMask.length; i++) {
      const v = bevMask[i] > 0 ? 255 : 0;
      imgData.data[i * 4] = v;
      imgData.data[i * 4 + 1] = v;
      imgData.data[i * 4 + 2] = v;
      imgData.data[i * 4 + 3] = 255;
    }
    ctx.putImageData(imgData, 0, 0);
  }

  /**
   * Draws the Pure Pursuit trajectory (the origin-anchored Hermite curve
   * from stepPurePursuitControl's `originAnchoredTrajectory`, which starts
   * exactly at the vehicle's own origin (0,0) by construction -- passing
   * `targetCenter` here instead would draw a line disconnected from the EGO
   * marker) plus the EGO marker itself, onto the BEV panel. Call this
   * *after* stepPurePursuitControl(), passing its `originAnchoredTrajectory`
   * and `farPt`; pass `trajectory: null` for the fallback case (still draws
   * the EGO marker, no line).
   */
  drawTrajectory(ctx, trajectory, farPt) {
    if (trajectory && trajectory.length >= 2) {
      const pathPx = trajectory.map(([x, y]) => this._metricToPx(x, y));
      ctx.strokeStyle = 'rgb(0,255,0)';
      ctx.lineWidth = 4;
      ctx.beginPath();
      let started = false;
      for (const [u, v] of pathPx) {
        if (u < 0 || u >= this.bevW || v < 0 || v >= this.bevH) {
          started = false;
          continue;
        }
        if (!started) {
          ctx.moveTo(u, v);
          started = true;
        } else {
          ctx.lineTo(u, v);
        }
      }
      ctx.stroke();

      // Lookahead marker at the exact (x_f, y_f) point stepPurePursuitControl
      // actually steers toward, not an approximated index into the sampled path.
      if (farPt) {
        const [fu, fv] = this._metricToPx(farPt[0], farPt[1]);
        if (fu >= 0 && fu < this.bevW && fv >= 0 && fv < this.bevH) {
          ctx.fillStyle = 'rgb(50,255,0)'; // BGR (0,255,50) -> RGB
          ctx.beginPath();
          ctx.arc(fu, fv, 8, 0, Math.PI * 2);
          ctx.fill();
        }
      }
    }

    const egoU = Math.floor(this.bevW / 2);
    const egoV = Math.floor(this.vNear);
    ctx.fillStyle = 'rgb(0,100,255)';
    ctx.beginPath();
    ctx.arc(egoU, egoV, 9, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = 'rgb(255,255,255)';
    ctx.font = '12px sans-serif';
    ctx.fillText('EGO', egoU - 15, egoV + 20);
  }
}

// Port of yolop_lane_detector.py's draw_lane_lines: 50/50 alpha-blends
// green (0,255,0) onto the camera image wherever ll_seg_mask==1, in place.
export function applyLaneOverlay(imageData, maskUint8, alpha = 0.5) {
  const { data } = imageData;
  for (let i = 0; i < maskUint8.length; i++) {
    if (maskUint8[i] !== 1) continue;
    const idx = i * 4;
    data[idx] = data[idx] * (1 - alpha);
    data[idx + 1] = data[idx + 1] * (1 - alpha) + 255 * alpha;
    data[idx + 2] = data[idx + 2] * (1 - alpha);
  }
}

// ---------------------------------------------------------------------------
// Pure Pursuit control law port (bev_pure_pursuit_node.py's
// _run_control_step / _handle_fallback -- note the node imports
// utils/pure_pursuit.py's AdaptivePurePursuit class but never actually calls
// it; this inline preview-curvature + cross-track law is what really runs).
// ---------------------------------------------------------------------------
export function stepPurePursuitControl(targetCenter, currentOmega, dt, params) {
  const { lookaheadDistance, targetSpeed, angularGain, crossTrackGain, maxAngularSpeed, maxAngularAccel } = params;
  const maxDeltaW = maxAngularAccel * dt;

  if (!targetCenter || targetCenter.length < 2) {
    // Exact port of _handle_fallback, including its assignment (not
    // increment) of current_omega to the clamped delta.
    const newOmega = clamp(0.0 - currentOmega, -maxDeltaW, maxDeltaW);
    return { v: 0.0, omega: newOmega, farPt: null, crossTrackError: null };
  }

  const xs = targetCenter.map((p) => p[0]);
  const ys = targetCenter.map((p) => p[1]);
  const farIdx = argminAbs(xs, lookaheadDistance);
  const xF = xs[farIdx];
  const yF = ys[farIdx];

  let slope;
  if (farIdx > 0 && farIdx < xs.length - 1) {
    slope = (ys[farIdx + 1] - ys[farIdx - 1]) / Math.max(0.1, xs[farIdx + 1] - xs[farIdx - 1]);
  } else {
    slope = yF / Math.max(1.0, xF);
  }

  const crossTrackError = ys[0];
  const lSq = xF * xF + yF * yF;
  const previewCurvature = lSq > 1e-4 ? (2.0 * yF) / lSq : 0.0;
  const omegaPreview = angularGain * targetSpeed * previewCurvature;
  const omegaCrosstrack = crossTrackGain * Math.atan2(crossTrackError, Math.max(0.5, targetSpeed));

  let targetOmega = omegaPreview + omegaCrosstrack;
  targetOmega = clamp(targetOmega, -maxAngularSpeed, maxAngularSpeed);

  const deltaW = clamp(targetOmega - currentOmega, -maxDeltaW, maxDeltaW);
  const newOmega = currentOmega + deltaW;

  // Origin-anchored Hermite-spline trajectory, published as the
  // target_trajectory Path topic (this is separate from the anchor-offset
  // `targetCenter` line drawn onto the BEV panel above -- Python computes
  // it directly in bev_pure_pursuit_node.py, not in BEVLaneExtractor).
  const originAnchoredTrajectory = [];
  const denom = Math.max(1.0, xF * xF);
  const a2 = (3.0 * yF - xF * slope) / denom;
  const a3 = (xF * slope - 2.0 * yF) / (denom * xF);
  const trajX = linspace(0.0, xs[xs.length - 1], 50);
  for (let i = 0; i < trajX.length; i++) {
    const xVal = trajX[i];
    let yVal;
    if (xVal <= xF) {
      yVal = a3 * xVal * xVal * xVal + a2 * xVal * xVal;
    } else {
      yVal = ys[argminAbs(xs, xVal)];
    }
    originAnchoredTrajectory.push([xVal, yVal]);
  }

  return { v: targetSpeed, omega: newOmega, farPt: [xF, yF], crossTrackError, slope, originAnchoredTrajectory };
}
