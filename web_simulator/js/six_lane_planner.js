// 6レーン動的選択走行: src/oit_navigation/oit_navigation/6lane/six_lane_core.py の移植
// (同じパラメータ名・同じ計算). 地図なし・オドメトリなしで、毎フレームの白線
// (左/中央/右, base_link) だけから
//   1. 仮想6レーン (左白線〜中央線を3等分 = レーン1〜3, 中央線〜右白線を3等分 = レーン4〜6)
//   2. 前方の曲率プロファイル (近 3m / 中 6m / 遠 9.5m)
//   3. 現在の横位置 (レーン座標 F: 左白線=0, 中央線=3, 右白線=6) と現在レーン
// を求め、MLP (six_lane_policy.json, train_policy.py で学習) が速度とコース形状から
// 6レーンの確率を出す。コーンで塞がれたレーンを除外し、ヒステリシスで目標レーンを確定、
// 進入角制限つき Pure Pursuit で目標レーン中心へ追従する。
//
// 左回りコース想定: 左カーブのアウト = レーン6、イン = レーン1。直線ではアウト側 (homeLane=6)。

export const N_LANES = 6;
const ROLES = ['left', 'center', 'right'];
export const STRAIGHT = 'STRAIGHT', ENTRY = 'ENTRY', APEX = 'APEX', EXIT = 'EXIT', LOST = 'LOST';

export const SIX_LANE_PARAMS = {
  // 知覚
  stations: [3.0, 6.0, 9.5], stationMargin: -0.5, minPointX: 1.0, binWidth: 0.5, minPoints: 4, minSpan: 2.5, cubicRidge: 0.05,
  // xPos: 横位置を測る前方距離 [m] (そこでの車線の向きで車軸位置へ戻す)。0 (車軸) で直接測ると
  // 2次式を点群の手前まで外挿することになり、カーブ入口で切片がずれて横位置を大きく誤る
  kappaAlpha: [0.35, 0.3, 0.2], kappaDecay: 0.9, xPos: 1.5,
  // 現在の横位置 F の追跡: 白線の役割取り違えで F が1フレームで跳ぶのを弾く [レーン, -, s]
  lateralGate: 0.35, lateralGain: 0.5, lateralResyncTime: 3.0,
  // 判断 (学習時と共通)
  vMax: 1.5, kappaScale: 10.0, kappaStraight: 0.015, kappaCurve: 0.06, homeLane: 6,
  // コミット層
  switchMargin: 0.12, switchFrames: 5, switchFramesPerLane: 2, coneXMin: -1.0, coneXMax: 8.0, coneClearance: 0.75, coneBlockFactor: 0.02,
  conePassMargin: 1.2,
  // 制御
  lookaheadMin: 2.0, lookaheadMax: 3.5, lookaheadTime: 1.5, maxApproachAngle: (22 * Math.PI) / 180, edgeClearance: 0.75,
  maxAngularSpeed: 1.2, maxAngularAccel: 4.0, aLatMax: 1.2, vMin: 0.5, accel: 0.8, decel: 1.5, confAlpha: 0.1, lostTimeout: 0.8,
};

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

// ---------------------------------------------------------------------------
// 知覚: 曲率プロファイル
// ---------------------------------------------------------------------------
function solve(M, b) {
  const n = b.length;
  const A = M.map((row, i) => [...row, b[i]]);
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(A[r][col]) > Math.abs(A[piv][col])) piv = r;
    [A[col], A[piv]] = [A[piv], A[col]];
    const pv = A[col][col];
    if (Math.abs(pv) < 1e-12) return null;
    for (let k = col; k <= n; k++) A[col][k] /= pv;
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = A[r][col];
      for (let k = col; k <= n; k++) A[r][k] -= f * A[col][k];
    }
  }
  return A.map((row) => row[n]);
}

export function fitCubic(xs, ys, ridge) {
  const AtA = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
  const Atb = [0, 0, 0, 0];
  for (let i = 0; i < xs.length; i++) {
    const x = xs[i], w = 1 / Math.max(x, 1);
    const row = [w, x * w, x * x * w, x * x * x * w];
    for (let a = 0; a < 4; a++) {
      Atb[a] += row[a] * ys[i] * w;
      for (let b = 0; b < 4; b++) AtA[a][b] += row[a] * row[b];
    }
  }
  AtA[2][2] += ridge;
  AtA[3][3] += ridge * 10;
  return solve(AtA, Atb);
}

/** minPointX より先の点を binWidth ごとに平均する (画像の行で等間隔 = 近くに密集、を前方距離方向に均す)。 */
export function binPoints(px, py, p) {
  const bins = new Map();
  for (let i = 0; i < px.length; i++) {
    if (px[i] < p.minPointX) continue;
    const k = Math.floor(px[i] / p.binWidth);
    const b = bins.get(k) || [0, 0, 0];
    b[0] += px[i]; b[1] += py[i]; b[2] += 1;
    bins.set(k, b);
  }
  const keys = [...bins.keys()].sort((a, b) => a - b);
  return { xs: keys.map((k) => bins.get(k)[0] / bins.get(k)[2]), ys: keys.map((k) => bins.get(k)[1] / bins.get(k)[2]) };
}

export function cubicCurvature(a, x) {
  const d1 = a[1] + 2 * a[2] * x + 3 * a[3] * x * x;
  const d2 = 2 * a[2] + 6 * a[3] * x;
  return d2 / Math.pow(1 + d1 * d1, 1.5);
}

/** 各ステーションの符号付き曲率 (左カーブ正)。観測できないステーションは null。 */
export function measureCurvatures(lines, p) {
  const sums = p.stations.map(() => 0);
  const wsum = p.stations.map(() => 0);
  for (const role of ROLES) {
    const ln = lines[role];
    if (!ln || !ln.detected || !ln.px) continue;
    const { xs: bx, ys: by } = binPoints(ln.px, ln.py, p);
    if (bx.length < p.minPoints) continue;
    const lo = Math.min(...bx), hi = Math.max(...bx);
    if (hi - lo < p.minSpan) continue;
    const a = fitCubic(bx, by, p.cubicRidge);
    if (!a) continue;
    p.stations.forEach((xs, i) => {
      if (xs < lo - p.stationMargin || xs > hi + p.stationMargin) return;
      sums[i] += bx.length * cubicCurvature(a, xs);
      wsum[i] += bx.length;
    });
  }
  return sums.map((s, i) => (wsum[i] > 0 ? s / wsum[i] : null));
}

/** 目標レーンを from から to へ切り替えるのに必要な連続フレーム数 (大移動ほど慎重に)。 */
export const requiredSwitchFrames = (p, from, to) => p.switchFrames + p.switchFramesPerLane * (Math.abs(to - from) - 1);

/** 見えないステーション (カーブで白線が視野外へ出た等) は、手前で最後に見えた曲率が続くとみなす。 */
export function fillUnobserved(meas) {
  let last = null;
  return meas.map((m) => (last = m !== null ? m : last));
}

// ---------------------------------------------------------------------------
// 知覚: レーン座標
// ---------------------------------------------------------------------------
/** 点 (x, y) のレーン座標 F (左白線=0, 中央線=3, 右白線=6。範囲外は線形外挿)。 */
export function laneCoordinate(lines, x, y) {
  const { left: L, center: C, right: R } = lines;
  if (!L || !C || !R) return null;
  const yl = L.yAt(x), yc = C.yAt(x), yr = R.yAt(x);
  if (yl - yc < 0.3 || yc - yr < 0.3) return null;
  if (y >= yc) return (3 * (yl - y)) / (yl - yc);
  return 3 + (3 * (yc - y)) / (yc - yr);
}

/** レーン座標 F の x における横位置 y (laneCoordinate の逆)。 */
export function laneY(lines, x, F) {
  const yl = lines.left.yAt(x), yc = lines.center.yAt(x), yr = lines.right.yAt(x);
  if (F <= 3) return yl + ((yc - yl) * F) / 3;
  return yc + ((yr - yc) * (F - 3)) / 3;
}

const subLaneWidth = (lines, x, F) => {
  const yl = lines.left.yAt(x), yc = lines.center.yAt(x), yr = lines.right.yAt(x);
  return Math.max((F <= 3 ? yl - yc : yc - yr) / 3, 1e-3);
};

/** レーン座標 F の平行線の x における傾き dy/dx (= 車に対する車線の向き)。 */
export const laneSlope = (lines, x, F) => (laneY(lines, x + 0.25, F) - laneY(lines, x - 0.25, F)) / 0.5;

/**
 * 車軸 (base_link 原点) のレーン座標。点群のある xEval で車の正面の点のレーン座標を測り、
 * そこでの車線の向き psi を使って車軸位置へ戻す (正面の点は車軸から xEval*sin(psi) だけ横にずれている)。
 */
export function vehicleLaneCoordinate(lines, xEval) {
  const Fp = laneCoordinate(lines, xEval, 0);
  if (Fp === null || xEval === 0) return Fp;
  const psi = Math.atan(laneSlope(lines, xEval, Fp));
  return Fp - (xEval * Math.sin(psi)) / subLaneWidth(lines, xEval, Fp);
}

export const laneOf = (F) => clamp(Math.floor(F) + 1, 1, N_LANES);

// ---------------------------------------------------------------------------
// 判断: 特徴量・MLP・局面判定 (教師ルールと同じ)
// ---------------------------------------------------------------------------
export function features(v, kappas, F, conf, p) {
  const k = kappas.map((kk) => kk * p.kappaScale);
  return [v / p.vMax, k[0], k[1], k[2], (F - 3) / 3, conf].map((x) => clamp(x, -2, 2));
}

export class LanePolicyNet {
  constructor({ W1, b1, W2, b2 }) {
    Object.assign(this, { W1, b1, W2, b2 });
  }

  static async load(url) {
    const res = await fetch(url);
    if (!res.ok) throw new Error(`${url}: HTTP ${res.status}`);
    return new LanePolicyNet(await res.json());
  }

  forward(x) {
    const h = this.W1.map((row, i) => Math.tanh(row.reduce((acc, w, j) => acc + w * x[j], this.b1[i])));
    const z = this.W2.map((row, i) => row.reduce((acc, w, j) => acc + w * h[j], this.b2[i]));
    const zMax = Math.max(...z);
    const e = z.map((zi) => Math.exp(zi - zMax));
    const sum = e.reduce((a, b) => a + b, 0);
    return { probs: e.map((ei) => ei / sum), hidden: h };
  }
}

const dCurve = (kappa, p) => clamp((Math.abs(kappa) - p.kappaStraight) / (p.kappaCurve - p.kappaStraight), 0, 1);

/** 局面判定。{phase, sign(+1=左カーブ), target(連続レーン), intensity} */
export function classifyPhase(v, kappas, p) {
  const [kn, km, kf] = kappas;
  const dominant = [kn, km, kf].reduce((a, b) => (Math.abs(b) > Math.abs(a) ? b : a));
  if (Math.abs(dominant) < p.kappaStraight) return { phase: STRAIGHT, sign: 0, target: p.homeLane, intensity: 0 };
  const s = dominant > 0 ? 1 : -1;
  const same = (k) => (k * s > 0 ? dCurve(k, p) : 0);
  const cIn = 0.6 * same(kn) + 0.4 * same(km);
  const cUp = same(kf);
  const outer = s > 0 ? 6 : 1;
  const inner = s > 0 ? 1 : 6;
  const sf = clamp(0.5 + (0.5 * v) / p.vMax, 0.5, 1);
  if (cIn >= 0.45 && cUp >= 0.35) {
    const depth = sf * Math.min(1, 1.2 * cIn);
    return { phase: APEX, sign: s, target: outer + (inner - outer) * depth, intensity: cIn };
  }
  if (cIn >= 0.45) return { phase: EXIT, sign: s, target: outer, intensity: cIn };
  if (cUp >= 0.35) return { phase: ENTRY, sign: s, target: outer, intensity: cUp };
  return { phase: STRAIGHT, sign: s, target: p.homeLane, intensity: 0 };
}

// ---------------------------------------------------------------------------
// プランナー本体
// ---------------------------------------------------------------------------
export class SixLanePlanner {
  constructor(net, params = SIX_LANE_PARAMS) {
    this.p = { ...params };
    this.net = net;
    this.reset();
  }

  reset() {
    this.s = { kappas: [0, 0, 0], targetLane: null, pendingLane: null, pendingCount: 0, vCmd: 0, omegaCmd: 0, lostTime: 0, confEma: 1, blockHold: new Map(), FFilt: null, rejectTime: 0 };
    this.last = { phase: LOST, v: 0, omega: 0, lostTime: 0 };
  }

  /**
   * 1制御周期。
   * @param {number} dt [s]
   * @param {{left, center, right}|null} lines 各線 {yAt(x), detected, px?, py?} (base_link)。null = 観測なし
   * @param {number} vMeas 車輪速 (CAN) 由来の速度 [m/s]
   * @param {Array<{x:number, y:number}>} cones base_link のコーン位置
   * @param {boolean} reanchored 白線の追跡側が根拠 (3本揃い・二重線) をもって役割を付け直したフレーム。横位置の跳びを受け入れる
   */
  step(dt, lines, vMeas, cones = [], reanchored = false) {
    const p = this.p, s = this.s;
    const FMeas = lines ? vehicleLaneCoordinate(lines, p.xPos) : null;
    if (FMeas === null) {
      s.lostTime += dt;
      if (s.lostTime >= p.lostTimeout) { // 短い欠落は直前の指令を維持, 続けば減速停止
        s.vCmd = Math.max(0, s.vCmd - p.decel * dt);
        s.omegaCmd = this._rate(s.omegaCmd, 0, dt);
        s.FFilt = null; // 長く見失ったら横位置は観測から取り直す
      }
      this.last = { ...this.last, phase: LOST, v: s.vCmd, omega: s.omegaCmd, lostTime: s.lostTime };
      return this.last;
    }
    s.lostTime = 0;
    // 現在の横位置 (追跡済み)。shift = 白線の役割取り違えによる観測のずれ [レーン]。
    // 白線の形 (傾き・曲がり) は取り違えても平行なので使い続け、横位置だけ shift で補正する。
    if (reanchored) s.FFilt = null; // 追跡側の付け直しは根拠があるので、取り違えとみなさず観測から取り直す
    const { F: F0, rejected } = this._trackLateral(lines, FMeas, vMeas, dt);
    const shift = FMeas - F0;

    const meas = measureCurvatures(lines, p);
    fillUnobserved(meas).forEach((m, i) => {
      s.kappas[i] = m !== null ? s.kappas[i] + p.kappaAlpha[i] * (m - s.kappas[i]) : s.kappas[i] * p.kappaDecay;
    });
    const conf = ROLES.filter((r) => lines[r] && lines[r].detected).length / 3;
    const curLane = laneOf(F0);
    s.confEma += p.confAlpha * (conf - s.confEma); // 速度用 (1フレームの欠落で速度を揺らさない)

    const x = features(vMeas, s.kappas, F0, conf, p);
    const { probs: nnProbs, hidden } = this.net.forward(x);
    const blocked = this._blockedLanes(lines, cones, Math.max(vMeas, 0) * dt, shift);
    const probs = nnProbs.map((q, i) => (blocked.has(i + 1) ? q * p.coneBlockFactor : q));
    const sum = probs.reduce((a, b) => a + b, 0);
    for (let i = 0; i < probs.length; i++) probs[i] /= sum;

    if (s.targetLane === null) s.targetLane = curLane;
    const best = probs.indexOf(Math.max(...probs)) + 1;
    if (blocked.has(s.targetLane) && !blocked.has(best)) {
      Object.assign(s, { targetLane: best, pendingLane: null, pendingCount: 0 });
    } else if (best !== s.targetLane && probs[best - 1] - probs[s.targetLane - 1] > p.switchMargin) {
      s.pendingCount = s.pendingLane === best ? s.pendingCount + 1 : 1;
      s.pendingLane = best;
      if (s.pendingCount >= requiredSwitchFrames(p, s.targetLane, best)) Object.assign(s, { targetLane: best, pendingLane: null, pendingCount: 0 });
    } else {
      s.pendingLane = null;
      s.pendingCount = 0;
    }

    const vNow = Math.max(vMeas, 0);
    const Ld = clamp(vNow * p.lookaheadTime, p.lookaheadMin, p.lookaheadMax);
    const Ft = this._targetCoordinate(lines, Ld, s.targetLane);
    const yCur = laneY(lines, Ld, FMeas); // 今の横位置を保った場合の注視点 (白線の平行線なので shift に依らない)
    const yTgt = laneY(lines, Ld, Ft + shift);
    const maxDy = Ld * Math.tan(p.maxApproachAngle);
    const ty = yCur + clamp(yTgt - yCur, -maxDy, maxDy);
    const tx = Ld;
    const kappaPP = (2 * ty) / (tx * tx + ty * ty);

    const kappaPath = Math.max(Math.abs(s.kappas[0]), Math.abs(s.kappas[1]), Math.abs(kappaPP));
    let vTarget = Math.min(p.vMax, Math.sqrt(p.aLatMax / Math.max(kappaPath, 1e-3)));
    vTarget = Math.max(vTarget, p.vMin) * (0.6 + 0.4 * s.confEma);
    s.vCmd = vTarget > s.vCmd ? Math.min(vTarget, s.vCmd + p.accel * dt) : Math.max(vTarget, s.vCmd - p.decel * dt);
    const omega = clamp(Math.max(vNow, s.vCmd) * kappaPP, -p.maxAngularSpeed, p.maxAngularSpeed);
    s.omegaCmd = this._rate(s.omegaCmd, omega, dt);

    const ph = classifyPhase(vMeas, s.kappas, p);
    this.last = {
      phase: ph.phase, sign: ph.sign, teacherTarget: ph.target, intensity: ph.intensity,
      F: F0, FMeas, lateralRejected: rejected, currentLane: curLane, targetLane: s.targetLane, pendingLane: s.pendingLane, pendingCount: s.pendingCount,
      kappas: [...s.kappas], kappasMeas: meas, confidence: conf, features: x, hidden, nnProbs, probs,
      blocked: [...blocked].sort((a, b) => a - b), lookahead: [tx, ty], v: s.vCmd, omega: s.omegaCmd, lostTime: 0,
    };
    return this.last;
  }

  // 横位置 F を、白線に対する車の向き (平行線の傾き) と速度で予測し、観測で補正する。
  // 予測から lateralGate 以上離れた観測は白線の取り違えとみなして捨てる。
  _trackLateral(lines, FMeas, v, dt) {
    const p = this.p, s = this.s;
    if (s.FFilt === null) {
      s.FFilt = FMeas;
      s.rejectTime = 0;
      return { F: FMeas, rejected: false };
    }
    const FPred = s.FFilt + (laneSlope(lines, p.xPos, FMeas) * v * dt) / subLaneWidth(lines, p.xPos, FMeas); // 車線が左へ向いている (slope>0) = 車は右 (F 増) へずれていく
    const r = FMeas - FPred;
    if (Math.abs(r) <= p.lateralGate) {
      s.FFilt = FPred + p.lateralGain * r;
      s.rejectTime = 0;
      return { F: s.FFilt, rejected: false };
    }
    s.rejectTime += dt;
    if (s.rejectTime >= p.lateralResyncTime) {
      s.FFilt = FMeas;
      s.rejectTime = 0;
      return { F: FMeas, rejected: false };
    }
    s.FFilt = FPred;
    return { F: FPred, rejected: true };
  }

  // 目標レーンのレーン座標 (中心)。端のレーンは白線から edgeClearance 以上離す
  // (レーン中心は白線から 0.58m しかなく、車体がはみ出しやすい)。
  _targetCoordinate(lines, x, lane) {
    const yl = lines.left.yAt(x), yc = lines.center.yAt(x), yr = lines.right.yAt(x);
    const fMin = this.p.edgeClearance / Math.max((yl - yc) / 3, 1e-3);
    const fMax = 6 - this.p.edgeClearance / Math.max((yc - yr) / 3, 1e-3);
    return clamp(lane - 0.5, fMin, fMax);
  }

  _rate(prev, target, dt) {
    const step = this.p.maxAngularAccel * Math.max(dt, 0);
    return prev + clamp(target - prev, -step, step);
  }

  // コーンで塞がれたレーン。見えなくなった後も、そのコーンが車体後端を抜ける距離だけ
  // 走るまで塞がれたままにする (走行距離は車輪速の積分。自己位置は使わない)。
  _blockedLanes(lines, cones, travelled, shift = 0) {
    const p = this.p, hold = this.s.blockHold;
    for (const [k, d] of [...hold]) {
      if (d - travelled <= 0) hold.delete(k);
      else hold.set(k, d - travelled);
    }
    for (const c of cones) {
      if (c.x < p.coneXMin || c.x > p.coneXMax) continue;
      for (let k = 1; k <= N_LANES; k++) {
        if (Math.abs(laneY(lines, c.x, k - 0.5 + shift) - c.y) < p.coneClearance) hold.set(k, Math.max(hold.get(k) || 0, c.x + p.conePassMargin));
      }
    }
    return new Set(hold.keys());
  }
}

// ---------------------------------------------------------------------------
// デバッグ表示 (日本語)
// ---------------------------------------------------------------------------
const PHASE_JA = {
  STRAIGHT: '直線', ENTRY: 'カーブ進入前', APEX: 'カーブ旋回中', EXIT: 'カーブ脱出', LOST: '白線ロスト',
};

/** 思考結果を日本語の説明文 (行の配列) にする。 */
export function explainJa(st, p = SIX_LANE_PARAMS) {
  if (!st || st.phase === LOST || st.currentLane === undefined) {
    const t = st ? st.lostTime || 0 : 0;
    return [`白線を見失っています (${t.toFixed(1)}s)`, t >= p.lostTimeout ? '→ 減速して停止します' : '→ 直前の指令を維持'];
  }
  const dir = st.sign > 0 ? '左' : st.sign < 0 ? '右' : '';
  const k = (v) => `${v >= 0 ? '+' : ''}${v.toFixed(3)}`;
  const lines = [];
  lines.push(`速度 ${st.v.toFixed(2)} m/s ／ 曲率 近${k(st.kappas[0])} 中${k(st.kappas[1])} 遠${k(st.kappas[2])} [1/m]`);
  let why;
  switch (st.phase) {
    case STRAIGHT:
      why = `前方は直線 → 外側のレーン${p.homeLane}で次のカーブに備える`;
      break;
    case ENTRY:
      why = `前方に${dir}カーブを検知 → アウト側(レーン${Math.round(st.teacherTarget)})に寄せて進入準備`;
      break;
    case APEX:
      why = `${dir}カーブ旋回中 (強さ${(st.intensity * 100).toFixed(0)}%) → イン側へ切り込む (目安レーン${st.teacherTarget.toFixed(1)})`;
      break;
    case EXIT:
      why = `${dir}カーブの出口が見えた → アウト側(レーン${Math.round(st.teacherTarget)})へ膨らんで加速`;
      break;
    default:
      why = '';
  }
  lines.push(`局面: ${PHASE_JA[st.phase]}　${why}`);
  if (st.blocked.length) lines.push(`コーン: レーン${st.blocked.join(',')} が塞がれているため除外`);
  const nnBest = st.nnProbs.indexOf(Math.max(...st.nnProbs)) + 1;
  let decision = `NN推奨 レーン${nnBest} (${(st.nnProbs[nnBest - 1] * 100).toFixed(0)}%)`;
  if (st.pendingLane) decision += ` → レーン${st.pendingLane}へ切替待ち ${st.pendingCount}/${requiredSwitchFrames(p, st.targetLane, st.pendingLane)}`;
  else if (st.currentLane !== st.targetLane) decision += ` → レーン${st.currentLane}からレーン${st.targetLane}へ移動中`;
  else decision += ' → 目標レーンを維持';
  lines.push(decision);
  lines.push(`白線検出の信頼度 ${(st.confidence * 100).toFixed(0)}% (3本中${Math.round(st.confidence * 3)}本)`);
  if (st.lateralRejected) {
    lines.push(`白線の割り当てが急に変化 (観測F=${st.FMeas.toFixed(2)}) → 取り違えとみなし横位置は予測値を使用`);
  }
  return lines;
}
