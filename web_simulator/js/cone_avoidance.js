// web_simulator/js/cone_avoidance.js
// コーン回避: 反応的ナッジ(毎フレーム)、1周目のコーン記憶、2周目レーシング
// ラインへの回避後処理、コーンランドマークによるオドメトリ補正。
// js/lane_navigator.js (実機のnavigator.py/course_map.py/raceline_qp.pyの
// 移植)本体は一切変更しない -- ここにある関数はすべて外側から結果を使う/
// 上書きするだけ。詳細はdocs/superpowers/specs/2026-09-22-cone-avoidance-design.md参照。

import {
  vehicleToWorld, rateLimit, correctedPoseSequence, RacelineFollower, yawDriftApplied, densifyClosed,
} from './lane_navigator.js';

// --- 7.1: 半径1.0m クラスタ包絡 & 外周円弧トレース回避 ---
export const KEEP_OUT_RADIUS = 1.0; // [m] 各コーン中心からの侵入禁止半径
// 回避経路の基準点も、要求どおりコーン中心から半径1.0mに保つ。
export const REACT_CLEARANCE = KEEP_OUT_RADIUS;
const REACT_LOOKAHEAD_X = 7.0;
const REACT_MIN_X = 0.2;
const REACT_MAX_OMEGA_BIAS = 0.85;
const REACT_SLOW_X = 1.8;
const REACT_SLOW_V = 0.6;
const CLUSTER_LINK_DISTANCE = KEEP_OUT_RADIUS * 2;

// 回避方向のロック (チャタリング防止) & 通過後スムーズ復帰状態
let lockedAvoidSign = 0; // -1: 右抜け, +1: 左抜け, 0: なし
let lastThreatTime = 0;

/**
 * 1.0mの禁止円が接するコーンを、連結成分ごとのクラスタにまとめる。
 * クラスタ内の禁止領域は「円の和集合」であり、回避時には各円の外周を
 * なぞるため、コーン間を禁止円を横切ってすり抜けることはない。
 */
export function clusterConeDetections(cones) {
  const remaining = new Set(cones.map((_, i) => i));
  const clusters = [];
  while (remaining.size) {
    const seed = remaining.values().next().value;
    remaining.delete(seed);
    const indices = [seed];
    for (let cursor = 0; cursor < indices.length; cursor++) {
      const a = cones[indices[cursor]];
      for (const i of [...remaining]) {
        const b = cones[i];
        if (Math.hypot(a.x - b.x, a.y - b.y) <= CLUSTER_LINK_DISTANCE) {
          remaining.delete(i);
          indices.push(i);
        }
      }
    }
    const members = indices.map((i) => cones[i]);
    clusters.push({
      cones: members,
      minX: Math.min(...members.map((c) => c.x)),
      centerY: members.reduce((sum, c) => sum + c.y, 0) / members.length,
    });
  }
  return clusters;
}

/**
 * navigator.step()が返したcmdに、コーン群(クラスタ)の半径1.0m侵入禁止円の
 * 外周円弧を沿うように抜ける操舵バイアスを加えて返す。
 * @param {{v:number, omega:number}} cmd
 * @param {Array<{x:number, y:number, conf:number}>} detections 車体フレーム
 * @param {number} prevBias 前回返したbias (レート制限のため)
 * @param {number} dt
 * @param {number} maxRate [rad/s^2] biasの変化率上限
 */
export function reactiveAvoid(cmd, detections, prevBias, dt, maxRate = 4.0) {
  const now = performance.now() / 1000;

  // 1. 有効範囲内のコーンを収集し、禁止円が接するものをクラスタ化する。
  const validCones = detections.filter(d => d.x >= REACT_MIN_X && d.x <= REACT_LOOKAHEAD_X && Math.abs(d.y) < 3.0);
  const clusters = clusterConeDetections(validCones);
  // 進路と重なるクラスタだけを対象にする。最も手前のものを先に抜ければ、
  // その後のクラスタについて次の制御周期で改めて正しい側を選べる。
  const threat = clusters
    .filter((cluster) => cluster.cones.some((c) => Math.abs(c.y) < REACT_CLEARANCE))
    .sort((a, b) => a.minX - b.minX)[0];

  if (threat) {
    lastThreatTime = now;
  }

  // 脅威が無い／通過済みなら、操舵バイアスを滑らかに戻す。
  if (!threat || (now - lastThreatTime > 0.8)) {
    lockedAvoidSign = 0;
    const bias = rateLimit(prevBias, 0, maxRate, dt);
    return {
      v: cmd.v,
      omega: cmd.omega + bias,
      bias,
      debug: `回避待機: 有効コーン ${validCones.length} 本 / クラスタ ${clusters.length} 個`,
    };
  }

  // 2. コース中心線に対して左(y>0)のクラスタは右へ、右(y<0)のクラスタは
  // 左へ抜ける。通過中はロックし、検出値の揺れで左右を往復しないようにする。
  // ただし次のクラスタが反対側にある場合は、そのクラスタに合わせて切り替える。
  const desiredAvoidSign = threat.centerY > 0 ? -1 : 1;
  if (lockedAvoidSign === 0 || lockedAvoidSign !== desiredAvoidSign) lockedAvoidSign = desiredAvoidSign;

  // 3. 手前クラスタだけに沿って円弧を作る。早めに外側へ寄せ、クラスタの
  // 円が現在の注視断面を横切るときはその円弧の接線側を目標にする。
  const minX = threat.minX;
  const closeSlow = minX < REACT_SLOW_X;
  const lookaheadX = Math.max(0.9, Math.min(minX, 3.5));

  // 4. クラスタを構成する各 1.0m 禁止円の外周円弧（車体中心に対しては
  // REACT_CLEARANCE）を計算する。
  // 注視点 x = lookaheadX における安全な横位置境界 yTarget を求める
  let requiredOffset = 0;
  for (const c of threat.cones) {
    const dx = lookaheadX - c.x;
    if (Math.abs(dx) < REACT_CLEARANCE) {
      // 円の方程式: dy = sqrt(R^2 - dx^2)
      const arcWidth = Math.sqrt(REACT_CLEARANCE * REACT_CLEARANCE - dx * dx);
      if (lockedAvoidSign > 0) {
        // 左抜け: コーン中心より左側 (c.y + arcWidth)
        const targetY = c.y + arcWidth;
        if (targetY > requiredOffset) requiredOffset = targetY;
      } else {
        // 右抜け: コーン中心より右側 (c.y - arcWidth)
        const targetY = c.y - arcWidth;
        if (targetY < requiredOffset) requiredOffset = targetY;
      }
    } else if (c.x > lookaheadX) {
      // 先行するコーンに対しても事前に外側へアプローチ
      const directTarget = lockedAvoidSign > 0 ? (c.y + REACT_CLEARANCE) : (c.y - REACT_CLEARANCE);
      if (lockedAvoidSign > 0 && directTarget > requiredOffset) requiredOffset = directTarget;
      if (lockedAvoidSign < 0 && directTarget < requiredOffset) requiredOffset = directTarget;
    }
  }

  // 5. 目標円弧点 (lookaheadX, requiredOffset) に向けた円弧追従 (Pure Pursuit)
  // 曲率: kappa = 2 * y / (x^2 + y^2)
  const distSq = lookaheadX * lookaheadX + requiredOffset * requiredOffset;
  const curvature = (2.0 * requiredOffset) / Math.max(distSq, 1.0);

  const v = closeSlow ? Math.min(cmd.v, REACT_SLOW_V) : cmd.v;
  let targetBias = curvature * Math.max(v, 0.9) * 1.3;
  targetBias = Math.max(-REACT_MAX_OMEGA_BIAS, Math.min(REACT_MAX_OMEGA_BIAS, targetBias));

  const bias = rateLimit(prevBias, targetBias, maxRate, dt);
  return {
    v,
    omega: cmd.omega + bias,
    bias,
    debug: `回避中: ${threat.cones.length} 本のクラスタを${lockedAvoidSign > 0 ? '左' : '右'}へ回避 | 禁止半径 ${KEEP_OUT_RADIUS.toFixed(1)} m | 横目標 ${requiredOffset.toFixed(2)} m | 操舵補正 ${bias.toFixed(2)} rad/s`,
  };
}

// --- 7.2: 1周目のコーン記憶 ---
export class ConeRecorder {
  constructor(gateRadius = 0.6) { this.cones = []; this.gateRadius = gateRadius; }

  /**
   * @param {number} s localizer.s (走行距離)
   * @param {[number, number, number]} pose [localizer.x, localizer.y, localizer.yaw]
   * @param {Array<{x:number, y:number}>} detections 車体フレーム
   */
  update(s, pose, detections) {
    for (const d of detections) {
      const [wx, wy] = vehicleToWorld(pose, d.x, d.y); // 名寄せ用の未補正ラフ座標
      const hit = this.cones.find((c) => Math.hypot(c.roughX - wx, c.roughY - wy) < this.gateRadius);
      if (!hit) {
        this.cones.push({ roughX: wx, roughY: wy, s, localX: d.x, localY: d.y });
      } else if (d.x < hit.localX) {
        hit.roughX = wx; hit.roughY = wy; hit.s = s; hit.localX = d.x; hit.localY = d.y;
      }
    }
  }

  /**
   * ラップ終了時に1回呼ぶ。境界点とまったく同じ補正 (方位ドリフト補正は地図が
   * 掛けたときだけ + ループ閉じ込み) で記録時の姿勢から再投影し、コースマップ
   * (= レーシングライン) と同じ座標にする。実機の lane_nav/cone_avoidance.py と同じ。
   * @param {Array<{s:number, pose:[number,number,number]}>} samples navigator.recorder.samples
   * @param {number} yawDrift navigator.yawDrift
   * @param {object|null} courseMap navigator.courseMap (closureOffsets を使う)
   * @param {object} lapParams navigator.p.lap
   * @returns {Array<{x:number, y:number}>}
   */
  finalize(samples, yawDrift, courseMap = null, lapParams = null) {
    if (samples.length === 0) return [];
    const poses = lapParams && !yawDriftApplied(lapParams, yawDrift)
      ? samples.map((sm) => [...sm.pose])
      : correctedPoseSequence(samples, yawDrift);
    return this.cones.map((c) => {
      let bi = 0, bd = Infinity;
      for (let i = 0; i < samples.length; i++) {
        const d = Math.abs(samples[i].s - c.s);
        if (d < bd) { bd = d; bi = i; }
      }
      let [x, y] = vehicleToWorld(poses[bi], c.localX, c.localY);
      if (courseMap && courseMap.closureOffsets && courseMap.s.length) {
        let k = 0, kd = Infinity;
        for (let i = 0; i < courseMap.s.length; i++) {
          const d = Math.abs(courseMap.s[i] - c.s);
          if (d < kd) { kd = d; k = i; }
        }
        x += courseMap.closureOffsets[k][0];
        y += courseMap.closureOffsets[k][1];
      }
      return { x, y };
    });
  }
}

// --- 7.3: 2周目レーシングラインの回避後処理 ---
// 実機の lane_nav/cone_avoidance.py deflect_raceline_around_cones() と同じ (定数・計算も同一)。
// 2 周目のラインはコーン中心から 1.3m 離す (反応的回避の禁止半径 1.0m より広い): 地図の誤差 (~0.3m) と
// 追従のショートカット (~0.2m) が重なっても車体がコーンに触れない余裕を持たせる。
export const DEFLECT_CLEARANCE = 1.3; // [m]
const DEFLECT_STEP = 0.2; // [m] 押し出す前に細かくする間隔 (RacelineFollower と同じ)
const DEFLECT_WINDOW = 4.5; // [m] コーンの前後この範囲に押し出しを配る
const DEFLECT_ITERATIONS = 4;
const DEFLECT_EDGE_MARGIN = 0.5; // [m] コース境界からこれだけ内側に保つ (車体半幅 0.4 + 0.1)

/**
 * QP のレーシングライン (閉曲線) を, 記憶したコーンから DEFLECT_CLEARANCE 離れるよう滑らかに押し出す。
 * ウェイポイントは直線部で数 m おきしかないので、先に細かくしてから最も近い点を横に押し、前後
 * DEFLECT_WINDOW に raised cosine の重みで配る (山形のこぶ)。left/right (コースマップの境界) があれば、
 * コーンの左右のうちコースに収まる側 (今のラインに近い方) へ押す。optimizeRaceline 自体は変更しない後処理。
 * @param {Array<[number,number]>} points 閉曲線、順序あり
 * @param {number[]} speeds pointsと同じ長さ
 * @param {Array<{x:number, y:number}>} cones finalize()の出力
 * @param {Array<[number,number]>|null} left コースマップの左境界
 * @param {Array<[number,number]>|null} right コースマップの右境界
 * @returns {{points: Array<[number,number]>, speeds: number[]}} 細かくした点列 (RacelineFollower にそのまま渡せる)
 */
export function deflectRacelineAroundCones(points, speeds, cones, left = null, right = null) {
  const { path, speed } = densifyClosed(points, speeds, DEFLECT_STEP);
  const out = path.map((p) => [...p]);
  const n = out.length;
  const half = Math.max(1, Math.round(DEFLECT_WINDOW / DEFLECT_STEP));
  const weights = [];
  for (let k = -half; k <= half; k++) weights.push(0.5 * (1 + Math.cos((Math.PI * k) / (half + 1))));
  const haveBounds = left && right && left.length >= 2;
  const mids = haveBounds ? left.map((l, i) => [(l[0] + right[i][0]) / 2, (l[1] + right[i][1]) / 2]) : null;
  for (let it = 0; it < DEFLECT_ITERATIONS; it++) {
    let moved = false;
    for (const cone of cones) {
      let i = -1, di = Infinity;
      for (let q = 0; q < n; q++) {
        const d = Math.hypot(out[q][0] - cone.x, out[q][1] - cone.y);
        if (d < di) { di = d; i = q; }
      }
      if (di >= DEFLECT_CLEARANCE - 1e-3) continue;
      let push;
      if (haveBounds) {
        let j = 0, dj = Infinity;
        for (let q = 0; q < mids.length; q++) {
          const d = Math.hypot(mids[q][0] - cone.x, mids[q][1] - cone.y);
          if (d < dj) { dj = d; j = q; }
        }
        const ax = left[j][0] - right[j][0], ay = left[j][1] - right[j][1];
        const width = Math.hypot(ax, ay);
        const nx = ax / Math.max(width, 1e-9), ny = ay / Math.max(width, 1e-9); // 右境界 -> 左境界
        const cLat = (cone.x - right[j][0]) * nx + (cone.y - right[j][1]) * ny;
        const pLat = (out[i][0] - right[j][0]) * nx + (out[i][1] - right[j][1]) * ny;
        const lo = DEFLECT_EDGE_MARGIN, hi = width - DEFLECT_EDGE_MARGIN;
        const sides = [cLat + DEFLECT_CLEARANCE, cLat - DEFLECT_CLEARANCE].filter((t) => t >= lo && t <= hi);
        let target;
        if (sides.length) target = sides.reduce((a, b) => (Math.abs(b - pLat) < Math.abs(a - pLat) ? b : a));
        else target = hi - cLat >= cLat - lo ? hi : lo; // どちらにも入れない: 広い側の限界まで
        if (Math.abs(target - pLat) < 1e-3) continue;
        push = [(target - pLat) * nx, (target - pLat) * ny];
      } else {
        const ux = (out[i][0] - cone.x) / Math.max(di, 1e-6), uy = (out[i][1] - cone.y) / Math.max(di, 1e-6);
        push = [(DEFLECT_CLEARANCE - di) * ux, (DEFLECT_CLEARANCE - di) * uy];
      }
      for (let k = -half; k <= half; k++) {
        const q = (((i + k) % n) + n) % n;
        const w = weights[k + half];
        out[q][0] += w * push[0];
        out[q][1] += w * push[1];
      }
      moved = true;
    }
    if (!moved) break;
  }
  return { points: out, speeds: speed };
}

/**
 * navigator.raceline/navigator.followerを、コーン回避済みのfollowerに
 * 差し替える。navigator.state === RACINGへの遷移直後に1回だけ呼ぶ。
 * @param {import('./lane_navigator.js').LaneNavigator} navigator
 * @param {Array<{x:number, y:number}>} cones
 * @param {object} trackerParams navigator.p.tracker
 * @returns {Array<[number,number]>} 押し出した (細かくした) ライン
 */
export function applyRacelineDeflection(navigator, cones, trackerParams) {
  const m = navigator.courseMap;
  const { points, speeds } = deflectRacelineAroundCones(navigator.raceline.points, navigator.raceline.speed, cones,
    m ? m.left : null, m ? m.right : null);
  navigator.follower = new RacelineFollower(points, speeds, trackerParams);
  return points;
}

// --- 7.4: 2周目、コーンランドマークによるオドメトリ補正 ---

/**
 * 検出中のコーンを記憶済みコーン地図と照合し、ズレをnavigatorに反映する。
 * navigator.state === RACINGの間、毎ティック呼ぶ。位置(x,y)のみ補正、
 * 姿勢は点ランドマーク1つからは決まらないため常に0を渡す。
 * @param {import('./lane_navigator.js').LaneNavigator} navigator
 * @param {[number, number, number]} localizerPose [localizer.x, localizer.y, localizer.yaw]
 * @param {Array<{x:number, y:number}>} detections 車体フレーム
 * @param {Array<{x:number, y:number}>} coneMapPoints finalize()の出力
 * @param {number} gate [m] この距離を超える照合は棄却する
 */
export function coneLandmarkCorrection(navigator, localizerPose, detections, coneMapPoints, gate = 1.0) {
  if (coneMapPoints.length === 0 || detections.length === 0) return;
  const mp = navigator.mapPose(localizerPose);
  let sumDx = 0, sumDy = 0, count = 0;
  for (const d of detections) {
    const [wx, wy] = vehicleToWorld(mp, d.x, d.y);
    let best = null, bestDist = Infinity;
    for (const c of coneMapPoints) {
      const dist = Math.hypot(c.x - wx, c.y - wy);
      if (dist < bestDist) { bestDist = dist; best = c; }
    }
    if (best && bestDist < gate) { sumDx += best.x - wx; sumDy += best.y - wy; count++; }
  }
  if (count > 0) navigator.applyExternalCorrection(sumDx / count, sumDy / count, 0);
}
