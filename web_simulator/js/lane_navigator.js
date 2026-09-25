// In-browser port of src/oit_navigation/oit_navigation/lane_nav/ (the
// ROS-independent core of the real vehicle's lane_detector +
// lane_navigator nodes). Each section below mirrors one Python module, in
// the same order, with the same parameter names/defaults, so this simulator
// drives exactly the algorithm the real vehicle runs:
//
//   geometry.py          -> CameraModel / projectToGround / fitLine / LineFit
//   mask_lines.py        -> extractMaskLines (YOLOP mask -> per-line points)
//   line_tracker.py      -> LineTracker (UFLD slot -> left/center/right role)
//   boundary_recorder.py -> BoundaryRecorder (lap 1: drop L/R boundary points,
//                           sparse on straights / dense in curves)
//   course_map.py        -> buildCourseMap (trim, yaw-drift + loop closure)
//   raceline_qp.py       -> optimizeRaceline (min-curvature QP, FISTA)
//   path_tracker.py      -> RacelineFollower
//   navigator.py         -> LaneNavigator (MAPPING -> RACING state machine)

// ---------------------------------------------------------------------------
// geometry.py
// ---------------------------------------------------------------------------
export const DEFAULT_CAMERA = {
  fx: 763.17, fy: 763.17, cx: 960.0, cy: 540.0, refWidth: 1920, refHeight: 1080,
  camHeight: 0.56, camX: 0.055, pitchDown: (7.3 * Math.PI) / 180,
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
// image point sequences, same shape as the UFLD decode ({u, v} top->bottom).
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

// ---------------------------------------------------------------------------
// boundary_recorder.py
// ---------------------------------------------------------------------------
export const RECORDER_PARAMS = {
  xRec: 2.0, dsStraight: 3.0, dsCurve: 0.6, kappaStraight: 0.03, kappaCurve: 0.20,
  maxHeadingStep: (12 * Math.PI) / 180, medianWindow: 5, kappaAlpha: 0.3, minSpeedForKappa: 0.3,
};

export function spacingForCurvature(kappa, p) {
  const k = Math.abs(kappa);
  if (k <= p.kappaStraight) return p.dsStraight;
  if (k >= p.kappaCurve) return p.dsCurve;
  const t = (k - p.kappaStraight) / (p.kappaCurve - p.kappaStraight);
  return p.dsStraight + t * (p.dsCurve - p.dsStraight);
}

export function vehicleToWorld(pose, x, y) {
  const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
  return [pose[0] + c * x - s * y, pose[1] + s * x + c * y];
}

function median(arr) {
  const a = [...arr].sort((p, q) => p - q);
  const m = a.length >> 1;
  return a.length % 2 ? a[m] : 0.5 * (a[m - 1] + a[m]);
}

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

export class BoundaryRecorder {
  constructor(params = RECORDER_PARAMS) {
    this.p = params; this.samples = []; this.bufL = []; this.bufR = [];
    this.kappa = 0; this.lastS = null; this.lastYaw = null;
  }

  nextSpacing() { return spacingForCurvature(this.kappa, this.p); }

  update(s, pose, v, omega, yLeft, yRight, leftDetected, rightDetected, lineKappa) {
    const p = this.p;
    let kMeas = v >= p.minSpeedForKappa ? Math.abs(omega) / v : 0;
    if (lineKappa !== null && lineKappa !== undefined) kMeas = Math.max(kMeas, Math.abs(lineKappa));
    this.kappa += p.kappaAlpha * (kMeas - this.kappa);

    if (yLeft !== null) { this.bufL.push(yLeft); if (this.bufL.length > p.medianWindow) this.bufL.shift(); }
    if (yRight !== null) { this.bufR.push(yRight); if (this.bufR.length > p.medianWindow) this.bufR.shift(); }
    if (!this.bufL.length || !this.bufR.length) return null;

    if (this.lastS !== null) {
      const ds = s - this.lastS;
      const dyaw = Math.abs(wrap(pose[2] - this.lastYaw));
      if (ds < this.nextSpacing() && dyaw < p.maxHeadingStep) return null;
      if (ds < 0.2) return null;
    }
    const yl = median(this.bufL), yr = median(this.bufR);
    const sample = {
      s, pose: [...pose],
      left: vehicleToWorld(pose, p.xRec, yl), right: vehicleToWorld(pose, p.xRec, yr),
      kappa: this.kappa, leftDetected, rightDetected, yLeft: yl, yRight: yr,
    };
    this.samples.push(sample);
    this.lastS = s;
    this.lastYaw = pose[2];
    return sample;
  }
}

// ---------------------------------------------------------------------------
// course_map.py
// ---------------------------------------------------------------------------
export const LAP_PARAMS = {
  minLapLength: 30.0, closeRadius: 2.5, closeHeading: Math.PI / 4,
  loopClosure: true, maxClosureCorrection: 1.5,
  yawDriftCorrection: false, maxYawDriftCorrection: (20 * Math.PI) / 180,
};

export function lapCompleted(samples, pose, sNow, startPose, startS, p) {
  if (samples.length < 5 || sNow - startS < p.minLapLength) return false;
  const d = Math.hypot(pose[0] - startPose[0], pose[1] - startPose[1]);
  return d <= p.closeRadius && Math.abs(wrap(pose[2] - startPose[2])) <= p.closeHeading;
}

// 補正後の姿勢列だけを計算する (境界点の再投影にもコーン記憶の再投影にも
// 使う共通ロジック)。ヨードリフトを走行距離sに応じて滑らかに配分するのは
// 元のcorrectYawDriftと完全に同じ計算。
export function correctedPoseSequence(samples, yawDrift) {
  const s0 = samples[0].s, s1 = samples[samples.length - 1].s;
  const span = Math.max(s1 - s0, 1e-9);
  const poses = [[...samples[0].pose]];
  for (let i = 1; i < samples.length; i++) {
    const a = samples[i - 1], b = samples[i];
    const rot = -yawDrift * ((a.s + b.s) * 0.5 - s0) / span;
    const dx = b.pose[0] - a.pose[0], dy = b.pose[1] - a.pose[1];
    const c = Math.cos(rot), s = Math.sin(rot);
    const prev = poses[poses.length - 1];
    poses.push([prev[0] + c * dx - s * dy, prev[1] + s * dx + c * dy, b.pose[2] - yawDrift * (b.s - s0) / span]);
  }
  return poses;
}

/**
 * 周回完了時点の odom -> map 補正 [tx, ty, theta]。2 周目の LaneNavigator.corr の初期値。
 * 方位ドリフト補正を掛けた地図は「ドリフトを取り除いた座標」なので、1 周分ドリフトした今の odom 姿勢を
 * そのまま地図上の位置として使うと 1 周目の終わりの断面で (位置 ~2m, 方位 = yawDrift) ずれる。
 * 最後の記録断面の odom 姿勢を補正後の姿勢に移す剛体変換を返す (補正しないときは恒等変換)。
 * course_map.py odom_to_map_correction() と同じ計算。
 */
export function odomToMapCorrection(samples, p, yawDrift) {
  if (!samples.length || !yawDriftApplied(p, yawDrift)) return [0, 0, 0];
  const po = samples[samples.length - 1].pose;
  const pc = correctedPoseSequence(samples, yawDrift)[samples.length - 1];
  const th = pc[2] - po[2];
  const c = Math.cos(th), s = Math.sin(th);
  return [pc[0] - (c * po[0] - s * po[1]), pc[1] - (s * po[0] + c * po[1]), th];
}

function correctYawDrift(samples, yawDrift, xRec) {
  const poses = correctedPoseSequence(samples, yawDrift);
  return {
    left: poses.map((ps, i) => vehicleToWorld(ps, xRec, samples[i].yLeft)),
    right: poses.map((ps, i) => vehicleToWorld(ps, xRec, samples[i].yRight)),
  };
}

/** buildCourseMap が方位ドリフト補正を掛けるか (コーン記憶も同じ条件で合わせる)。course_map.py yaw_drift_applied。 */
export function yawDriftApplied(p, yawDrift) {
  return p.yawDriftCorrection && yawDrift !== 0 && Math.abs(yawDrift) <= p.maxYawDriftCorrection;
}

export function buildCourseMap(samples, startPose, p, yawDrift = 0, xRec = 2.0) {
  let left = samples.map((s) => [...s.left]);
  let right = samples.map((s) => [...s.right]);
  if (yawDriftApplied(p, yawDrift)) {
    ({ left, right } = correctYawDrift(samples, yawDrift, xRec));
  }
  let center = left.map((l, i) => [(l[0] + right[i][0]) / 2, (l[1] + right[i][1]) / 2]);
  let ss = samples.map((s) => s.s);

  // Drop sections that overlap/overtake the first section.
  let t0 = [center[1][0] - center[0][0], center[1][1] - center[0][1]];
  const n0 = Math.max(Math.hypot(...t0), 1e-9);
  t0 = [t0[0] / n0, t0[1] / n0];
  let keep = samples.length;
  for (let i = samples.length - 1; i > (samples.length >> 1); i--) {
    const d = [center[i][0] - center[0][0], center[i][1] - center[0][1]];
    if (Math.hypot(...d) < 3.0 && d[0] * t0[0] + d[1] * t0[1] > -0.3) keep = i;
    else break;
  }
  left = left.slice(0, keep); right = right.slice(0, keep); center = center.slice(0, keep); ss = ss.slice(0, keep);
  const leftDetected = samples.slice(0, keep).map((s) => s.leftDetected);
  const rightDetected = samples.slice(0, keep).map((s) => s.rightDetected);

  let closure = 0;
  const n = center.length;
  // 各断面に掛けたループ閉じ込みの補正量 (1 周目に記憶したコーンを境界と同じ座標に載せるのに使う)
  let closureOffsets = center.map(() => [0, 0]);
  if (p.loopClosure && n >= 5) {
    let t = [center[n - 1][0] - center[n - 2][0], center[n - 1][1] - center[n - 2][1]];
    const tn = Math.max(Math.hypot(...t), 1e-9);
    t = [t[0] / tn, t[1] / tn];
    const d = [center[0][0] - center[n - 1][0], center[0][1] - center[n - 1][1]];
    const along = d[0] * t[0] + d[1] * t[1];
    const gap = Math.max(along, 1e-3);
    const err = [-(d[0] - along * t[0]), -(d[1] - along * t[1])];
    closure = Math.hypot(...err);
    if (closure <= p.maxClosureCorrection) {
      const denom = Math.max(ss[n - 1] - ss[0] + gap, 1e-9);
      const fix = (arr) => arr.map((q, i) => {
        const w = (ss[i] - ss[0]) / denom;
        return [q[0] - w * err[0], q[1] - w * err[1]];
      });
      left = fix(left); right = fix(right); center = fix(center);
      closureOffsets = fix(closureOffsets);
    }
  }
  return { left, right, center, s: ss, leftDetected, rightDetected, startPose: [...startPose], closureError: closure, closureOffsets };
}

// ---------------------------------------------------------------------------
// raceline_qp.py
// ---------------------------------------------------------------------------
export const RACELINE_PARAMS = {
  vehicleHalfWidth: 0.40, safetyMargin: 0.35, lambdaCenter: 1e-4, lambdaSmooth: 1e-3,
  outerIterations: 3, fistaIterations: 3000, fistaTol: 1e-9,
  vMax: 3.0, vMin: 0.8, aLatMax: 1.5, aAccel: 1.0, aDecel: 1.5,
};

function spacingClosed(pts) {
  const n = pts.length;
  const hNext = pts.map((q, i) => Math.max(Math.hypot(pts[(i + 1) % n][0] - q[0], pts[(i + 1) % n][1] - q[1]), 1e-3));
  const hPrev = hNext.map((_, i) => hNext[(i - 1 + n) % n]);
  return { hPrev, hNext };
}

export function solveBoxLsq(Q, c, lo, hi, x0, iters, tol) {
  // min 0.5 x'Qx - c'x  s.t. lo<=x<=hi  (FISTA), Q = A'A given directly.
  const n = c.length;
  const mv = (M, x) => M.map((row) => { let s = 0; for (let j = 0; j < n; j++) s += row[j] * x[j]; return s; });
  let v = new Array(n).fill(1 / Math.sqrt(n));
  for (let k = 0; k < 100; k++) {
    const w = mv(Q, v);
    const nrm = Math.hypot(...w);
    if (nrm < 1e-15) break;
    v = w.map((z) => z / nrm);
  }
  const Qv = mv(Q, v);
  const L = Math.max(v.reduce((s, z, i) => s + z * Qv[i], 0), 1e-12) * 1.01;
  const clip = (x) => x.map((z, i) => Math.min(hi[i], Math.max(lo[i], z)));
  let x = clip(x0);
  let y = [...x];
  let t = 1;
  for (let it = 0; it < iters; it++) {
    const g = mv(Q, y);
    const xNew = clip(y.map((z, i) => z - (g[i] - c[i]) / L));
    const tNew = 0.5 * (1 + Math.sqrt(1 + 4 * t * t));
    let maxStep = 0;
    for (let i = 0; i < n; i++) maxStep = Math.max(maxStep, Math.abs(xNew[i] - x[i]));
    y = xNew.map((z, i) => z + ((t - 1) / tNew) * (z - x[i]));
    x = xNew; t = tNew;
    if (maxStep < tol) break;
  }
  return x;
}

export function signedCurvature(pts) {
  const n = pts.length;
  return pts.map((p1, i) => {
    const p0 = pts[(i - 1 + n) % n], p2 = pts[(i + 1) % n];
    const a = Math.hypot(p1[0] - p0[0], p1[1] - p0[1]);
    const b = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
    const c = Math.hypot(p2[0] - p0[0], p2[1] - p0[1]);
    const cross = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]);
    return (2 * cross) / Math.max(a * b * c, 1e-9);
  });
}

export function speedProfile(pts, kappa, p) {
  const n = pts.length;
  const { hNext } = spacingClosed(pts);
  const v = kappa.map((k) => Math.max(p.vMin, Math.min(p.vMax, Math.sqrt(p.aLatMax / Math.max(Math.abs(k), 1e-6)))));
  for (let pass = 0; pass < 2; pass++) {
    for (let i = 0; i < n; i++) {
      const j = (i + 1) % n;
      v[j] = Math.min(v[j], Math.sqrt(v[i] ** 2 + 2 * p.aAccel * hNext[i]));
    }
    for (let i = n - 1; i >= 0; i--) {
      const j = (i + 1) % n;
      v[i] = Math.min(v[i], Math.sqrt(v[j] ** 2 + 2 * p.aDecel * hNext[i]));
    }
  }
  return v.map((z) => Math.max(z, p.vMin));
}

export function optimizeRaceline(left, right, p = RACELINE_PARAMS) {
  const n = left.length;
  if (n < 5) throw new Error(`断面が少なすぎます (${n} 点)`);
  const span = left.map((l, i) => [right[i][0] - l[0], right[i][1] - l[1]]);
  const lo = [], hi = [];
  for (let i = 0; i < n; i++) {
    const m = Math.min((p.vehicleHalfWidth + p.safetyMargin) / Math.max(Math.hypot(...span[i]), 1e-3), 0.5);
    lo.push(m); hi.push(1 - m);
  }
  let alpha = new Array(n).fill(0.5);
  for (let outer = 0; outer < p.outerIterations; outer++) {
    const pts = left.map((l, i) => [l[0] + alpha[i] * span[i][0], l[1] + alpha[i] * span[i][1]]);
    const { hPrev, hNext } = spacingClosed(pts);
    // Rows of A (sparse: 3 nonzeros for curvature rows) and b.
    const rows = [];
    for (let i = 0; i < n; i++) {
      const a = 2 / (hPrev[i] + hNext[i]);
      const coef = [[(i - 1 + n) % n, a / hPrev[i]], [i, -a * (1 / hPrev[i] + 1 / hNext[i])], [(i + 1) % n, a / hNext[i]]];
      for (const comp of [0, 1]) {
        const entries = coef.map(([j, d]) => [j, d * span[j][comp]]);
        const b = -coef.reduce((s, [j, d]) => s + d * left[j][comp], 0);
        rows.push({ entries, b });
      }
      rows.push({ entries: [[i, Math.sqrt(p.lambdaCenter)]], b: Math.sqrt(p.lambdaCenter) * 0.5 });
      rows.push({ entries: [[i, Math.sqrt(p.lambdaSmooth)], [(i - 1 + n) % n, -Math.sqrt(p.lambdaSmooth)]], b: 0 });
    }
    const Q = Array.from({ length: n }, () => new Array(n).fill(0));
    const c = new Array(n).fill(0);
    for (const { entries, b } of rows) {
      for (const [j, aj] of entries) {
        c[j] += aj * b;
        for (const [k, ak] of entries) Q[j][k] += aj * ak;
      }
    }
    alpha = solveBoxLsq(Q, c, lo, hi, alpha, p.fistaIterations, p.fistaTol);
  }
  const points = left.map((l, i) => [l[0] + alpha[i] * span[i][0], l[1] + alpha[i] * span[i][1]]);
  const kappa = signedCurvature(points);
  const speed = speedProfile(points, kappa, p);
  const { hNext } = spacingClosed(points);
  return { points, alpha, left, right, kappa, speed, length: hNext.reduce((a, b) => a + b, 0) };
}

// ---------------------------------------------------------------------------
// path_tracker.py
// ---------------------------------------------------------------------------
export const TRACKER_PARAMS = {
  lookaheadMin: 1.2, lookaheadMax: 2.5, lookaheadTime: 0.6, crossTrackLookaheadGain: 2.0,
  maxAngularSpeed: 1.5, maxAngularAccel: 4.0, mappingSpeed: 1.0, curveSlowdown: 0.5,
  curvatureFilterTau: 0.5,
};

export function arcCurvature(x, y) {
  const d2 = x * x + y * y;
  return d2 < 1e-6 ? 0 : (2 * y) / d2;
}

export function rateLimit(prev, target, maxRate, dt) {
  const step = maxRate * Math.max(dt, 0);
  return prev + Math.max(-step, Math.min(step, target - prev));
}

export function densifyClosed(points, values, step = 0.2) {
  const n = points.length;
  const outP = [], outV = [];
  for (let i = 0; i < n; i++) {
    const p0 = points[(i - 1 + n) % n], p1 = points[i], p2 = points[(i + 1) % n], p3 = points[(i + 2) % n];
    const seg = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
    const m = Math.max(1, Math.ceil(seg / step));
    for (let k = 0; k < m; k++) {
      const t = k / m, t2 = t * t, t3 = t2 * t;
      const cr = (d) => 0.5 * (2 * p1[d] + (-p0[d] + p2[d]) * t + (2 * p0[d] - 5 * p1[d] + 4 * p2[d] - p3[d]) * t2
        + (-p0[d] + 3 * p1[d] - 3 * p2[d] + p3[d]) * t3);
      outP.push([cr(0), cr(1)]);
      outV.push(values[i] + (values[(i + 1) % n] - values[i]) * t);
    }
  }
  return { path: outP, speed: outV };
}

export class RacelineFollower {
  constructor(points, speeds, params = TRACKER_PARAMS) {
    this.p = params;
    ({ path: this.path, speed: this.speed } = densifyClosed(points, speeds));
    const n = this.path.length;
    this.seg = this.path.map((q, i) => Math.hypot(this.path[(i + 1) % n][0] - q[0], this.path[(i + 1) % n][1] - q[1]));
    this.idx = null;
  }

  nearest(x, y) {
    const n = this.path.length;
    let best = Infinity, bi = 0;
    const range = this.idx === null ? [...Array(n).keys()] : Array.from({ length: 100 }, (_, k) => (this.idx - 20 + k + n) % n);
    for (const i of range) {
      const d = Math.hypot(this.path[i][0] - x, this.path[i][1] - y);
      if (d < best) { best = d; bi = i; }
    }
    this.idx = bi;
    return bi;
  }

  target(pose, vNow) {
    const [x, y, yaw] = pose;
    const i = this.nearest(x, y);
    const n = this.path.length;
    const cross = Math.hypot(this.path[i][0] - x, this.path[i][1] - y);
    let la = Math.min(Math.max(vNow * this.p.lookaheadTime, this.p.lookaheadMin), this.p.lookaheadMax);
    la += this.p.crossTrackLookaheadGain * cross;
    let j = i, acc = 0;
    while (acc < la) { acc += this.seg[j]; j = (j + 1) % n; }
    const dx = this.path[j][0] - x, dy = this.path[j][1] - y;
    const c = Math.cos(yaw), s = Math.sin(yaw);
    return { speed: this.speed[i], tx: c * dx + s * dy, ty: -s * dx + c * dy, index: i, cross };
  }
}

// ---------------------------------------------------------------------------
// navigator.py
// ---------------------------------------------------------------------------
export const MAPPING = 'MAPPING', OPTIMIZING = 'OPTIMIZING', RACING = 'RACING', STOPPED = 'STOPPED';

export const NAVIGATOR_PARAMS = {
  recorder: RECORDER_PARAMS, lap: LAP_PARAMS, raceline: RACELINE_PARAMS, tracker: TRACKER_PARAMS,
  headingWindow: 15, linesTimeout: 0.8, linesHoldDistance: 3.0, linesLostSpeed: 0.5, stopDecel: 1.5,
  mapMatchingGain: 0.1, mapMatchingMaxError: 0.8, matchXMin: 1.0, matchXMax: 6.0, matchXStep: 1.0,
  matchDamping: [2.0, 2.0, 20.0],
  matchMaxStepXy: 0.03, matchMaxStepYaw: 0.005, matchMinLines: 2,
};

function nearestSegment(poly, px, py) {
  const n = poly.length;
  let best = Infinity, bi = -1;
  for (let i = 0; i < n; i++) {
    const a = poly[i], b = poly[(i + 1) % n];
    const abx = b[0] - a[0], aby = b[1] - a[1];
    const L2 = Math.max(abx * abx + aby * aby, 1e-12);
    const t = Math.min(1, Math.max(0, ((px - a[0]) * abx + (py - a[1]) * aby) / L2));
    const d2 = (px - a[0] - t * abx) ** 2 + (py - a[1] - t * aby) ** 2;
    if (d2 < best) { best = d2; bi = i; }
  }
  const a = poly[bi], b = poly[(bi + 1) % n];
  const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
  if (L < 1e-6) return null;
  const nx = -(b[1] - a[1]) / L, ny = (b[0] - a[0]) / L;
  return { n: [nx, ny], dist: nx * (px - a[0]) + ny * (py - a[1]) };
}

export class LaneNavigator {
  constructor(params = NAVIGATOR_PARAMS) { this.p = params; this.reset(); }

  reset() {
    this.state = MAPPING;
    this.recorder = new BoundaryRecorder(this.p.recorder);
    this.startPose = null;
    this.startS = 0;
    this.yawUnwrapped = null;
    this.lastYaw = 0;
    this.yawDrift = 0;
    this.lineHeadingBuf = [];
    this.startHeadingSamples = [];
    this.courseMap = null;
    this.raceline = null;
    this.follower = null;
    this.lap = 1;
    this.cmd = { v: 0, omega: 0 };
    this.curvFilt = null; // 1周目の減速用曲率 (ローパス後)
    this.corr = [0, 0, 0];
    this.corrInit = [0, 0, 0]; // RACING 開始時の corr (方位ドリフト補正した地図に今の姿勢を合わせる)
    this.lastLinesTime = null;
    this.lastIndex = null;
    this.lapStartS = 0;
    this.sNow = 0;
    this.centerAnchor = null; // 最後に見えた中央線と、その時の odom 姿勢・走行距離 {fit, pose, s}
    this.linesLost = false;
    this.message = '';
  }

  finishMapping() {
    if (this.state !== MAPPING || this.recorder.samples.length < 5) return false;
    this._buildAndOptimize();
    return true;
  }

  step(now, dt, pose, vMeas, omegaMeas, s, lines) {
    if (lines && lines.lines.center) this.lastLinesTime = now;
    this.sNow = s;
    let cmd;
    if (this.state === MAPPING) cmd = this._stepMapping(now, dt, pose, vMeas, omegaMeas, s, lines);
    else if (this.state === RACING) cmd = this._stepRacing(dt, pose, vMeas, lines, s);
    else cmd = this._stop(dt);
    this.cmd = cmd;
    return cmd;
  }

  _stepMapping(now, dt, pose, vMeas, omegaMeas, s, lines) {
    const tp = this.p.tracker;
    if (this.yawUnwrapped === null) this.yawUnwrapped = pose[2];
    else this.yawUnwrapped += wrap(pose[2] - this.lastYaw);
    this.lastYaw = pose[2];

    if (this.startPose === null) {
      if (!lines || !lines.lines.center) return this._stop(dt);
      this.startPose = [...pose];
      this.startS = s;
    }
    if (lines && lines.detected.center) {
      const h = this.yawUnwrapped + lines.lines.center.headingAt(0.5);
      this.lineHeadingBuf.push(h);
      if (this.lineHeadingBuf.length > this.p.headingWindow) this.lineHeadingBuf.shift();
      if (this.startHeadingSamples.length < this.p.headingWindow) this.startHeadingSamples.push(h);
    }
    if (lines) {
      const { left: L, center: C, right: R } = lines.lines;
      const xr = this.p.recorder.xRec;
      this.recorder.update(s, pose, vMeas, omegaMeas, L ? L.yAt(xr) : null, R ? R.yAt(xr) : null,
        !!lines.detected.left, !!lines.detected.right, C ? C.curvatureAt(xr) : null);
      if (C) this.centerAnchor = { fit: C, pose: [...pose], s };
    }
    if (lapCompleted(this.recorder.samples, pose, s, this.startPose, this.startS, this.p.lap)) {
      if (this.startHeadingSamples.length >= 3 && this.lineHeadingBuf.length >= 3) {
        const turned = median(this.lineHeadingBuf) - median(this.startHeadingSamples);
        const k = Math.round(turned / (2 * Math.PI));
        if (k !== 0) this.yawDrift = turned - 2 * Math.PI * k;
      }
      this._buildAndOptimize();
      if (this.state === RACING) return this._stepRacing(dt, pose, vMeas, lines, s);
    }
    const cmd = this._centerTracking(now, dt, pose, vMeas, s);
    if (!this.linesLost) {
      this.message = `1周目 記録中: 断面 ${this.recorder.samples.length} 点 / 間隔 ${this.recorder.nextSpacing().toFixed(1)}m`;
    }
    return cmd;
  }

  // 中央白線のレーントラッキング (1 周目)。navigator.py の _center_tracking() と同じ計算。
  // 最後に見えた中央線 (その時の車体座標のフィット) を、それ以降のオドメトリの移動分だけ
  // 座標変換して追う。検出が途切れ途切れの場所 (コーナー等) で毎回減速停止→再加速を
  // 繰り返すと実機が前後にガクガクするため、白線ロスト (linesTimeout 超過) 後も
  // linesHoldDistance [m] までは linesLostSpeed 以下で記憶した線を走り、それでも
  // 見えなければ減速停止する。
  _centerTracking(now, dt, pose, vMeas, s) {
    const tp = this.p.tracker;
    const lost = this.lastLinesTime === null || now - this.lastLinesTime > this.p.linesTimeout;
    this.linesLost = lost;
    if (!this.centerAnchor) return this._stop(dt);
    const { fit: C, pose: ap, s: aS } = this.centerAnchor;
    if (lost && s - aS > this.p.linesHoldDistance) {
      this.message = '白線ロスト: 減速停止';
      return this._stop(dt);
    }
    // 現在の車体位置を、中央線を観測した時の車体座標で表す
    const dx = pose[0] - ap[0], dy = pose[1] - ap[1];
    const ca = Math.cos(ap[2]), sa = Math.sin(ap[2]);
    const px = ca * dx + sa * dy, py = -sa * dx + ca * dy;
    const dyaw = wrap(pose[2] - ap[2]);
    const la = Math.min(Math.max(vMeas * tp.lookaheadTime, tp.lookaheadMin), tp.lookaheadMax);
    const xa = Math.max(px + la, C.xMin);
    const ya = C.yAt(xa);
    // 注視点を現在の車体座標へ
    const cy = Math.cos(dyaw), sy = Math.sin(dyaw);
    const tx = xa - px, ty = ya - py;
    const kappa = arcCurvature(cy * tx + sy * ty, -sy * tx + cy * ty);
    // フレーム毎の中央線フィットの曲率は検出ノイズで揺れるため、そのまま速度にすると
    // 目標速度が毎フレーム上下し前後にガクガクする。曲率をローパスし、速度も 2 周目と
    // 同じ加減速制限 (raceline.aAccel / aDecel) で変化させる (navigator.py と同じ)。
    const curv = Math.abs(C.curvatureAt(xa));
    if (this.curvFilt === null || !(tp.curvatureFilterTau > 0)) this.curvFilt = curv;
    else this.curvFilt += (curv - this.curvFilt) * Math.min(1, dt / tp.curvatureFilterTau);
    let vTarget = tp.mappingSpeed / (1 + tp.curveSlowdown * this.curvFilt * 4);
    if (lost) {
      vTarget = Math.min(vTarget, this.p.linesLostSpeed);
      this.message = '白線ロスト: 記憶した中央線で走行';
    }
    const rp = this.p.raceline;
    const v = rateLimit(this.cmd.v, vTarget, vTarget > this.cmd.v ? rp.aAccel : rp.aDecel, dt);
    let omega = Math.max(-tp.maxAngularSpeed, Math.min(tp.maxAngularSpeed, v * kappa));
    omega = rateLimit(this.cmd.omega, omega, tp.maxAngularAccel, dt);
    return { v, omega };
  }

  _buildAndOptimize() {
    this.state = OPTIMIZING;
    try {
      this.courseMap = buildCourseMap(this.recorder.samples, this.startPose, this.p.lap, this.yawDrift, this.p.recorder.xRec);
      this.corrInit = odomToMapCorrection(this.recorder.samples, this.p.lap, this.yawDrift);
      this._startRacing();
    } catch (e) {
      this.state = STOPPED;
      this.message = `レーシングライン生成失敗: ${e.message}`;
    }
  }

  _startRacing() {
    const m = this.courseMap;
    this.raceline = optimizeRaceline(m.left, m.right, this.p.raceline);
    this.follower = new RacelineFollower(this.raceline.points, this.raceline.speed, this.p.tracker);
    this.corr = [...this.corrInit];
    this.state = RACING;
    this.lap = 2;
    this.lastIndex = null;
    this.lapStartS = this.sNow;
    this.message = `2周目以降: ウェイポイント ${this.raceline.points.length} 点 / 1周 ${this.raceline.length.toFixed(1)}m`;
  }

  mapPose(pose) {
    const [tx, ty, th] = this.corr;
    const c = Math.cos(th), s = Math.sin(th);
    return [tx + c * pose[0] - s * pose[1], ty + s * pose[0] + c * pose[1], pose[2] + th];
  }

  _stepRacing(dt, pose, vMeas, lines, sNow) {
    const tp = this.p.tracker;
    if (lines && this.p.mapMatchingGain > 0) this._mapMatching(pose, lines);
    const mp = this.mapPose(pose);
    const tgt = this.follower.target(mp, vMeas);
    if (this.lastIndex !== null && tgt.index < this.lastIndex - (this.follower.path.length >> 1)
      && sNow - this.lapStartS > 0.7 * this.raceline.length) {
      this.lap++;
      this.lapStartS = sNow;
    }
    this.lastIndex = tgt.index;
    const rp = this.p.raceline;
    const v = rateLimit(this.cmd.v, tgt.speed, tgt.speed > this.cmd.v ? rp.aAccel : rp.aDecel, dt);
    let omega = Math.max(-tp.maxAngularSpeed, Math.min(tp.maxAngularSpeed, Math.max(v, 0.3) * arcCurvature(tgt.tx, tgt.ty)));
    omega = rateLimit(this.cmd.omega, omega, tp.maxAngularAccel, dt);
    return { v, omega };
  }

  _mapMatching(pose, lines) {
    const m = this.courseMap;
    const pp = this.p;
    const mp = this.mapPose(pose);
    const c = Math.cos(mp[2]), s = Math.sin(mp[2]);
    const J = [], r = [], w = [];
    let linesUsed = 0;
    for (const [role, poly, weight] of [['left', m.left, 1.0], ['right', m.right, 1.0], ['center', m.center, 0.5]]) {
      const fit = lines.lines[role];
      if (!fit || !lines.detected[role]) continue;
      const xLo = Math.max(fit.xMin, pp.matchXMin), xHi = Math.min(fit.xMax, pp.matchXMax);
      const nBefore = r.length;
      for (let x = xLo; x <= xHi + 1e-6; x += pp.matchXStep) {
        const y = fit.yAt(x);
        const px = mp[0] + c * x - s * y, py = mp[1] + s * x + c * y;
        const hit = nearestSegment(poly, px, py);
        if (!hit || Math.abs(hit.dist) > pp.mapMatchingMaxError) continue;
        J.push([hit.n[0], hit.n[1], hit.n[0] * -(py - mp[1]) + hit.n[1] * (px - mp[0])]);
        r.push(hit.dist);
        w.push(weight);
      }
      if (r.length > nBefore) linesUsed++;
    }
    if (r.length < 3 || linesUsed < pp.matchMinLines) return;
    const H = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    const g = [0, 0, 0];
    for (let k = 0; k < r.length; k++) {
      for (let a = 0; a < 3; a++) {
        g[a] += J[k][a] * w[k] * r[k];
        for (let b = 0; b < 3; b++) H[a][b] += J[k][a] * w[k] * J[k][b];
      }
    }
    for (let a = 0; a < 3; a++) H[a][a] += pp.matchDamping[a];
    const sol = solve3(H, g);
    if (!sol) return;
    const delta = sol.map((z) => -z * pp.mapMatchingGain);
    delta[0] = Math.max(-pp.matchMaxStepXy, Math.min(pp.matchMaxStepXy, delta[0]));
    delta[1] = Math.max(-pp.matchMaxStepXy, Math.min(pp.matchMaxStepXy, delta[1]));
    delta[2] = Math.max(-pp.matchMaxStepYaw, Math.min(pp.matchMaxStepYaw, delta[2]));
    const nw = [mp[0] + delta[0], mp[1] + delta[1], mp[2] + delta[2]];
    const th = nw[2] - pose[2];
    const ct = Math.cos(th), st = Math.sin(th);
    this.corr = [nw[0] - (ct * pose[0] - st * pose[1]), nw[1] - (st * pose[0] + ct * pose[1]), th];
  }

  // navigator.py本体にはない外部フック: cone_avoidance.jsのコーンランドマーク
  // 照合結果をthis.corrに反映する。_mapMatching()と違い、呼び出し側
  // (coneLandmarkCorrection)が渡すdx/dyは既にmapPose座標系(this.corrを
  // 適用した後の世界座標系)での差分なので、_mapMatching()のような
  // 「生のpose座標からの逆算」は不要で、corrへの直接加算でよい。
  // _mapMatching自体は一切変更しない。
  applyExternalCorrection(dx, dy, dyaw, damping = 0.15) {
    this.corr = [
      this.corr[0] + dx * damping,
      this.corr[1] + dy * damping,
      this.corr[2] + dyaw * damping,
    ];
  }

  _stop(dt) {
    const v = Math.max(0, this.cmd.v - this.p.stopDecel * dt);
    return { v, omega: v === 0 ? 0 : this.cmd.omega * 0.9 };
  }

  status() {
    return {
      state: this.state, lap: this.lap, samples: this.recorder.samples.length,
      spacing: +this.recorder.nextSpacing().toFixed(2), kappa: +this.recorder.kappa.toFixed(3),
      v: +this.cmd.v.toFixed(2), omega: +this.cmd.omega.toFixed(3),
      correction: this.corr.map((z) => +z.toFixed(3)), yaw_drift: +this.yawDrift.toFixed(4),
      message: this.message,
    };
  }
}
