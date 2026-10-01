// In-browser port of src/oit_navigation/oit_navigation/lane_core/ (the
// ROS-independent core of the real vehicle's lane_detector node). Each section
// below mirrors one Python module, with the same parameter names/defaults:
//
//   geometry.py          -> CameraModel / projectToGround / fitLine / LineFit
//   mask_lines.py        -> extractMaskLines (YOLOP mask -> per-line points)
//   line_tracker.py      -> LineTracker (detector slot -> left/center/right role)
//
// (走行は six_lane_planner.js)

// ---------------------------------------------------------------------------
// geometry.py
// ---------------------------------------------------------------------------
// 実機 ZED X (SN47800407) の camera_info (640x360 の値 x3). シミュレータの車載カメラもこの内部パラメータで描画する
// (simulator.js applyOnboardIntrinsics). geometry.py CameraModel / navigation_params.yaml と同じ値
export const DEFAULT_CAMERA = {
  fx: 733.26, fy: 733.26, cx: 980.22, cy: 516.63, refWidth: 1920, refHeight: 1080,
  camHeight: 0.56, camX: 0.055, pitchDown: (1.8 * Math.PI) / 180,
};

export function projectToGround(cam, u, v, width, height, minDepression = Math.PI / 180) {
  const sx = width / cam.refWidth;
  const sy = height / cam.refHeight;
  const fx = cam.fx * sx, fy = cam.fy * sy, cx = cam.cx * sx, cy = cam.cy * sy;
  const s = Math.sin(cam.pitchDown), c = Math.cos(cam.pitchDown);
  const xs = [], ys = [];
  for (let i = 0; i < u.length; i++) {
    const xc = (u[i] - cx) / fx;
    const yc = (v[i] - cy) / fy;
    const rayFwd = c - yc * s;
    const rayDown = s + yc * c;
    if (!(rayDown > Math.tan(minDepression) * Math.max(rayFwd, 1e-6))) continue;
    const t = cam.camHeight / rayDown;
    xs.push(cam.camX + t * rayFwd);
    ys.push(-t * xc);
  }
  return { x: xs, y: ys };
}

export class LineFit {
  constructor(coeffs, xMin, xMax, nPoints, inferred = false) {
    this.c = coeffs; this.xMin = xMin; this.xMax = xMax; this.n = nPoints; this.inferred = inferred;
  }
  yAt(x) { return this.c[0] + this.c[1] * x + this.c[2] * x * x; }
  headingAt(x) { return Math.atan(this.c[1] + 2 * this.c[2] * x); }
  curvatureAt(x) {
    const d1 = this.c[1] + 2 * this.c[2] * x;
    return (2 * this.c[2]) / Math.pow(1 + d1 * d1, 1.5);
  }
  shifted(dy) { return new LineFit([this.c[0] + dy, this.c[1], this.c[2]], this.xMin, this.xMax, 0, true); }
}

function solve3(M, b) {
  // Gaussian elimination with partial pivoting (3x3 / small systems)
  const n = b.length;
  const A = M.map((row, i) => [...row, b[i]]);
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(A[r][col]) > Math.abs(A[piv][col])) piv = r;
    [A[col], A[piv]] = [A[piv], A[col]];
    const p = A[col][col];
    if (Math.abs(p) < 1e-12) return null;
    for (let k = col; k <= n; k++) A[col][k] /= p;
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = A[r][col];
      for (let k = col; k <= n; k++) A[r][k] -= f * A[col][k];
    }
  }
  return A.map((row) => row[n]);
}

export function fitLine(x, y, xMaxFit = 12.0, minPoints = 3, curvatureReg = 0.1) {
  const xs = [], ys = [];
  for (let i = 0; i < x.length; i++) if (x[i] > 0.3 && x[i] < xMaxFit) { xs.push(x[i]); ys.push(y[i]); }
  if (xs.length < minPoints) return null;
  const xMin = Math.min(...xs), xMax = Math.max(...xs);
  if (xMax - xMin < 0.8) return null;
  const AtA = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  const Atb = [0, 0, 0];
  for (let i = 0; i < xs.length; i++) {
    const w = 1 / Math.max(xs[i], 1);
    const row = [w, xs[i] * w, xs[i] * xs[i] * w];
    for (let a = 0; a < 3; a++) {
      Atb[a] += row[a] * ys[i] * w;
      for (let b = 0; b < 3; b++) AtA[a][b] += row[a] * row[b];
    }
  }
  AtA[2][2] += curvatureReg * 10;
  const c = solve3(AtA, Atb);
  return c ? new LineFit(c, xMin, xMax, xs.length) : null;
}

// ---------------------------------------------------------------------------
// mask_lines.py: white-line mask (YOLOP ll_seg, 0/1 row-major) -> per-line
// image point sequences, {u, v} top->bottom per line.
// ---------------------------------------------------------------------------
export const MASK_LINES_PARAMS = {
  topRatio: 0.45, rowStepRatio: 1 / 72, minRunPxRatio: 0.002, maxRunWidthRatio: 0.12,
  gateRatio: 0.10, maxGapRows: 4, minPoints: 4,
};

export function extractMaskLines(mask, w, h, p = MASK_LINES_PARAMS) {
  const top = Math.floor(h * p.topRatio);
  const step = Math.max(1, Math.round(h * p.rowStepRatio));
  const chains = [];
  let ri = -1;
  for (let v = h - 1; v > top; v -= step) {
    ri++;
    const depth = (v - top) / Math.max(h - 1 - top, 1);
    const scale = 0.4 + 0.6 * depth;
    const maxRun = p.maxRunWidthRatio * w * scale;
    const gate = Math.max(3, p.gateRatio * w * scale);
    const centers = [];
    const base = v * w;
    let start = -1;
    for (let u = 0; u <= w; u++) {
      const on = u < w && mask[base + u] > 0;
      if (on && start < 0) start = u;
      if (!on && start >= 0) {
        const len = u - start;
        if (len >= p.minRunPxRatio * w && len <= maxRun) centers.push((start + u - 1) / 2);
        start = -1;
      }
    }
    if (!centers.length) continue;
    const cands = [];
    chains.forEach((ch, ci) => {
      if (ri - ch.lastI > p.maxGapRows) return;
      let pred = ch.u[ch.u.length - 1];
      if (ch.u.length >= 2) pred += (ch.u[ch.u.length - 1] - ch.u[ch.u.length - 2]) * (ri - ch.lastI);
      centers.forEach((c, k) => {
        const d = Math.abs(c - pred);
        if (d <= gate) cands.push([d, ci, k]);
      });
    });
    cands.sort((a, b) => a[0] - b[0] || a[1] - b[1] || a[2] - b[2]);
    const takenChain = new Set();
    const used = new Set();
    for (const [, ci, k] of cands) {
      if (takenChain.has(ci) || used.has(k)) continue;
      chains[ci].u.push(centers[k]);
      chains[ci].v.push(v);
      chains[ci].lastI = ri;
      takenChain.add(ci);
      used.add(k);
    }
    centers.forEach((c, k) => { if (!used.has(k)) chains.push({ u: [c], v: [v], lastI: ri }); });
  }
  return chains.filter((ch) => ch.u.length >= p.minPoints)
    .map((ch) => ({ u: ch.u.slice().reverse(), v: ch.v.slice().reverse() }));
}

// ---------------------------------------------------------------------------
// line_tracker.py
// ---------------------------------------------------------------------------
export const ROLES = ['left', 'center', 'right'];

// 「車両は中央線の上」とは仮定しない (line_tracker.py の docstring 参照): 起動時の横位置は initOffset
// (中央線からの横ずれ, 左正)、見失い続けたら最後の横位置を保って並びと道幅だけ戻す、3本 (または
// 左右の境界2本) が道幅どおりに揃って見え続けたら付け直す (再アンカー)、2本しか見えないときは二重線
// (境界線) を手掛かりにする (車に近い方が境界)、外部から seedLanePosition() で置き直せる。
export const LINE_TRACKER_PARAMS = {
  xRef: 2.0, laneWidthInit: 3.5, laneWidthMin: 1.0, laneWidthMax: 5.0,
  widthAlpha: 0.1, gateRatio: 0.4, widthTolerance: 0.3, maxHeadingDiff: 0.3, lostResetFrames: 30,
  initOffset: 0.0, anchorTolerance: 0.15, anchorFrames: 3, anchorMaxXMin: 6.0, anchorHeadingDiff: 0.1,
  useDoubleLine: true, doubleLineGapMin: 0.4, doubleLineGapMax: 1.1,
};

export class LineTracker {
  constructor(params = LINE_TRACKER_PARAMS) { this.p = { ...params }; this.reset(); }

  reset() {
    const w = this.p.laneWidthInit;
    this.laneW = { left: w, right: w };
    this._setCenter(-this.p.initOffset); // 中央線は車両から見て -initOffset の位置
    this.lostFrames = 0;
    this.anchorShift = null;
    this.anchorCount = 0;
  }

  _setCenter(center) {
    this.offsets = { left: center + this.laneW.left, center, right: center - this.laneW.right };
  }

  /** 車両のレーン座標 F (左白線=0, 中央線=3, 右白線=6; 6レーン走行の横位置) から線の位置を置き直す。 */
  seedLanePosition(F) {
    const w = F <= 3 ? this.laneW.left : this.laneW.right;
    this._setCenter(((F - 3) * w) / 3);
    this.anchorShift = null;
    this.anchorCount = 0;
  }

  update(fitsIn) {
    const p = this.p;
    const fits = fitsIn.filter(Boolean);
    const meas = fits.map((f) => f.yAt(p.xRef));
    const heads = fits.map((f) => f.headingAt(p.xRef));
    let assign = this._assign(meas, heads);
    const anchored = this._reanchor(meas, heads, fits.map((f) => f.xMin));
    if (anchored) assign = anchored;
    this._preferInnerBoundaries(assign, meas, heads);
    const detected = {};
    for (const r of ROLES) detected[r] = assign[r] !== null && assign[r] !== undefined;

    if (!ROLES.some((r) => detected[r])) {
      this.lostFrames++;
      if (this.lostFrames >= p.lostResetFrames) {
        // 中央線の上とは仮定しない: 最後の横位置 (中央線の位置) を保って並びと道幅だけ戻す
        const center = this.offsets.center;
        this.laneW = { left: p.laneWidthInit, right: p.laneWidthInit };
        this._setCenter(center);
        this.lostFrames = 0;
      }
      return { lines: { left: null, center: null, right: null }, detected, offsets: { ...this.offsets }, laneWidths: { ...this.laneW } };
    }
    this.lostFrames = 0;

    const lines = { left: null, center: null, right: null };
    for (const r of ROLES) {
      if (detected[r]) { lines[r] = fits[assign[r]]; this.offsets[r] = meas[assign[r]]; }
    }
    for (const [side, a, b] of [['left', 'left', 'center'], ['right', 'center', 'right']]) {
      if (detected[a] && detected[b]) {
        const w = this.offsets[a] - this.offsets[b];
        if (w >= p.laneWidthMin && w <= p.laneWidthMax) this.laneW[side] += p.widthAlpha * (w - this.laneW[side]);
      }
    }
    const wl = this.laneW.left, wr = this.laneW.right;
    if (!lines.center) {
      if (lines.left) lines.center = lines.left.shifted(-wl);
      else if (lines.right) lines.center = lines.right.shifted(+wr);
    }
    if (!lines.left && lines.center) lines.left = lines.center.shifted(+wl);
    if (!lines.right && lines.center) lines.right = lines.center.shifted(-wr);
    for (const r of ROLES) if (!detected[r] && lines[r]) this.offsets[r] = lines[r].yAt(p.xRef);
    return { lines, detected, offsets: { ...this.offsets }, laneWidths: { ...this.laneW }, reanchored: !!anchored };
  }

  // Boundary = first line outside the center line: if an unused line with a
  // consistent lane width lies between the chosen boundary and the center,
  // take it instead (avoids locking onto the outer line of a double line).
  _preferInnerBoundaries(assign, meas, heads) {
    const p = this.p;
    const c = assign.center;
    if (c === null || c === undefined) return;
    const used = new Set(Object.values(assign).filter((v) => v !== null && v !== undefined));
    for (const [role, sign] of [['left', 1], ['right', -1]]) {
      const b = assign[role];
      if (b === null || b === undefined) continue;
      const w = this.laneW[role];
      let best = null;
      meas.forEach((m, k) => {
        if (used.has(k)) return;
        const d = sign * (m - meas[c]);
        if (d > 0 && d < sign * (meas[b] - meas[c]) && Math.abs(d - w) <= p.widthTolerance * w
          && Math.abs(heads[k] - heads[c]) <= p.maxHeadingDiff) {
          if (best === null || d < sign * (meas[best] - meas[c])) best = k;
        }
      });
      if (best !== null) {
        used.delete(b);
        used.add(best);
        assign[role] = best;
      }
    }
  }

  // 3本 (または左右の境界2本) が道幅どおりに揃って見えるのに追跡中の位置とゲート以上ずれていれば
  // (= 割り当てが線1本ぶんずれている)、それが anchorFrames 続いた時点でその割り当てを返す。
  _reanchor(meas, heads, xMins) {
    const p = this.p;
    const gate = Math.min(this.laneW.left, this.laneW.right) * p.gateRatio;
    const spacing = { '0,1': this.laneW.left, '1,2': this.laneW.right, '0,2': this.laneW.left + this.laneW.right };
    const opts = [null, ...meas.map((_, i) => i)];
    const candidates = []; // {prio, assign, shift}
    for (const a of opts) for (const b of opts) for (const c of opts) {
      if (a === null || c === null) continue; // 左右の境界が両方あるときだけ役割が一意に決まる
      const combo = [a, b, c];
      const used = combo.filter((v) => v !== null);
      if (new Set(used).size !== used.length || used.some((i) => xMins[i] > p.anchorMaxXMin)) continue;
      const ys = used.map((i) => meas[i]);
      let ordered = true;
      for (let k = 0; k + 1 < ys.length; k++) if (ys[k] <= ys[k + 1]) ordered = false;
      if (!ordered || !this._consistent(combo, meas, heads, spacing, p.anchorTolerance, p.anchorHeadingDiff)) continue;
      const cNew = b !== null ? meas[b] : meas[a] - this.laneW.left;
      candidates.push({ prio: b !== null ? 0 : 1, assign: { left: a, center: b, right: c }, shift: cNew - this.offsets.center });
    }
    if (!candidates.length && p.useDoubleLine) {
      for (let i = 0; i < meas.length; i++) for (let k = 0; k < meas.length; k++) {
        const near = meas[i], far = meas[k];
        // 同じ側 (車をまたがない) に平行に並び、i の方が車に近い
        if (near * far <= 0 || Math.abs(near) < 0.2 || !(Math.abs(near) < Math.abs(far))
          || Math.abs(far - near) < p.doubleLineGapMin || Math.abs(far - near) > p.doubleLineGapMax
          || Math.max(xMins[i], xMins[k]) > p.anchorMaxXMin
          || Math.abs(heads[i] - heads[k]) > p.anchorHeadingDiff) continue;
        const role = near < 0 ? 'right' : 'left';
        const cNew = role === 'right' ? near + this.laneW.right : near - this.laneW.left;
        candidates.push({ prio: 2, assign: { left: null, center: null, right: null, [role]: i }, shift: cNew - this.offsets.center });
      }
    }
    let best = null; // 3本 > 左右境界 > 二重線、次に今の追跡に近いもの
    for (const cand of candidates) {
      if (!best || cand.prio < best.prio || (cand.prio === best.prio && Math.abs(cand.shift) < Math.abs(best.shift))) best = cand;
    }
    if (!best || Math.abs(best.shift) <= gate) {
      this.anchorShift = null;
      this.anchorCount = 0;
      return null;
    }
    this.anchorCount = this.anchorShift !== null && Math.abs(best.shift - this.anchorShift) <= gate ? this.anchorCount + 1 : 1;
    this.anchorShift = best.shift;
    if (this.anchorCount < p.anchorFrames) return null;
    this.anchorShift = null;
    this.anchorCount = 0;
    return best.assign;
  }

  // 同時に割り当てる線どうしの間隔が道幅と整合し、向きが揃っているか。
  _consistent(combo, meas, heads, spacing, tolerance = this.p.widthTolerance, headingDiff = this.p.maxHeadingDiff) {
    for (let ra = 0; ra < 3; ra++) for (let rb = ra + 1; rb < 3; rb++) {
      if (combo[ra] === null || combo[rb] === null) continue;
      const expected = spacing[`${ra},${rb}`];
      if (Math.abs(meas[combo[ra]] - meas[combo[rb]] - expected) > tolerance * expected
        || Math.abs(heads[combo[ra]] - heads[combo[rb]]) > headingDiff) return false;
    }
    return true;
  }

  _assign(meas, heads) {
    const p = this.p;
    const gate = Math.min(this.laneW.left, this.laneW.right) * p.gateRatio;
    const spacing = { '0,1': this.laneW.left, '1,2': this.laneW.right, '0,2': this.laneW.left + this.laneW.right };
    const opts = [null, ...meas.map((_, i) => i)];
    let best = {};
    let bestScore = [-1, 0];
    for (const a of opts) for (const b of opts) for (const c of opts) {
      const combo = [a, b, c];
      const used = combo.filter((v) => v !== null);
      if (new Set(used).size !== used.length) continue;
      let ok = true, cost = 0;
      for (let k = 0; k < 3; k++) {
        if (combo[k] === null) continue;
        const d = Math.abs(meas[combo[k]] - this.offsets[ROLES[k]]);
        if (d > gate) { ok = false; break; }
        cost += d;
      }
      if (!ok) continue;
      const ys = used.map((i) => meas[i]);
      let ordered = true;
      for (let k = 0; k + 1 < ys.length; k++) if (ys[k] <= ys[k + 1]) ordered = false;
      if (!ordered) continue;
      // Lines assigned together must be spaced like the tracked lane widths
      // and roughly parallel (rejects junction/branch lines).
      if (!this._consistent(combo, meas, heads, spacing)) continue;
      const score = [used.length, -cost];
      if (score[0] > bestScore[0] || (score[0] === bestScore[0] && score[1] > bestScore[1])) {
        bestScore = score;
        best = { left: a, center: b, right: c };
      }
    }
    return best;
  }
}
