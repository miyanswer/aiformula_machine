// web_simulator/js/cone_avoidance.js
// コーン回避: 反応的ナッジ(毎フレーム)、1周目のコーン記憶、2周目レーシング
// ラインへの回避後処理、コーンランドマークによるオドメトリ補正。
// js/lane_navigator.js (実機のnavigator.py/course_map.py/raceline_qp.pyの
// 移植)本体は一切変更しない -- ここにある関数はすべて外側から結果を使う/
// 上書きするだけ。詳細はdocs/superpowers/specs/2026-09-22-cone-avoidance-design.md参照。

import { VEHICLE_HALF_WIDTH } from './collision.js';
import { CONE_RADIUS } from './cone_props.js';
import { vehicleToWorld, rateLimit, correctedPoseSequence, RacelineFollower } from './lane_navigator.js';

// --- 7.1: 反応的回避 ---
const REACT_MARGIN = 0.15;
export const REACT_CLEARANCE = VEHICLE_HALF_WIDTH + CONE_RADIUS + REACT_MARGIN; // 0.70
const REACT_LOOKAHEAD_X = 4.0;
const REACT_MIN_X = 0.3;
const REACT_GAIN = 1.5;
const REACT_MAX_OMEGA_BIAS = 0.6;
const REACT_SLOW_X = 1.5;
const REACT_SLOW_V = 0.5;

/**
 * navigator.step()が返したcmdに、検出中のコーンを避ける操舵バイアスを
 * 加えて返す。MAPPING/RACING両方で毎フレーム呼ぶ。
 * @param {{v:number, omega:number}} cmd
 * @param {Array<{x:number, y:number, conf:number}>} detections 車体フレーム
 * @param {number} prevBias 前回返したbias (レート制限のため)
 * @param {number} dt
 * @param {number} maxRate [rad/s^2] biasの変化率上限
 */
export function reactiveAvoid(cmd, detections, prevBias, dt, maxRate = 4.0) {
  let worstPush = 0, pushSign = 0, worstX = REACT_MIN_X, closeSlow = false;
  for (const d of detections) {
    if (d.x < REACT_MIN_X || d.x > REACT_LOOKAHEAD_X) continue;
    const shortfall = REACT_CLEARANCE - Math.abs(d.y);
    if (shortfall > worstPush) { worstPush = shortfall; pushSign = d.y >= 0 ? -1 : 1; worstX = d.x; }
    if (d.x < REACT_SLOW_X && Math.abs(d.y) < REACT_CLEARANCE) closeSlow = true;
  }
  let targetBias = 0;
  if (worstPush > 0) {
    targetBias = Math.max(-REACT_MAX_OMEGA_BIAS, Math.min(REACT_MAX_OMEGA_BIAS,
      pushSign * REACT_GAIN * worstPush / Math.max(worstX, 0.5)));
  }
  const bias = rateLimit(prevBias, targetBias, maxRate, dt);
  return {
    v: closeSlow ? Math.min(cmd.v, REACT_SLOW_V) : cmd.v,
    omega: cmd.omega + bias,
    bias,
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
   * ラップ終了時に1回呼ぶ。境界点と同じ「補正後姿勢列 + 記録時ローカル
   * オフセット」で再投影する。
   * @param {Array<{s:number, pose:[number,number,number]}>} samples navigator.recorder.samples
   * @param {number} yawDrift navigator.yawDrift
   * @returns {Array<{x:number, y:number}>}
   */
  finalize(samples, yawDrift) {
    if (samples.length === 0) return [];
    const poses = correctedPoseSequence(samples, yawDrift);
    return this.cones.map((c) => {
      let bi = 0, bd = Infinity;
      for (let i = 0; i < samples.length; i++) {
        const d = Math.abs(samples[i].s - c.s);
        if (d < bd) { bd = d; bi = i; }
      }
      const [x, y] = vehicleToWorld(poses[bi], c.localX, c.localY);
      return { x, y };
    });
  }
}

// --- 7.3: 2周目レーシングラインの回避後処理 ---
export const DEFLECT_CLEARANCE = VEHICLE_HALF_WIDTH + CONE_RADIUS + 0.20; // 0.75

/**
 * QP出力のレーシングライン点列を、記録済みコーンから離すよう局所的に
 * 押し出す。optimizeRaceline自体は変更しない後処理。
 * @param {Array<[number,number]>} points 閉曲線、順序あり
 * @param {number[]} speeds pointsと同じ長さ
 * @param {Array<{x:number, y:number}>} cones finalize()の出力
 * @returns {{points: Array<[number,number]>, speeds: number[]}}
 */
export function deflectRacelineAroundCones(points, speeds, cones) {
  const out = points.map((p) => [...p]);
  const n = out.length;
  for (const cone of cones) {
    let bestI = -1, bestD = Infinity;
    for (let i = 0; i < n; i++) {
      const d = Math.hypot(out[i][0] - cone.x, out[i][1] - cone.y);
      if (d < bestD) { bestD = d; bestI = i; }
    }
    if (bestD >= DEFLECT_CLEARANCE) continue;
    for (let k = -8; k <= 8; k++) {
      const i = ((bestI + k) % n + n) % n;
      const dx = out[i][0] - cone.x, dy = out[i][1] - cone.y;
      const dist = Math.max(Math.hypot(dx, dy), 1e-6);
      if (dist >= DEFLECT_CLEARANCE) continue;
      const push = (DEFLECT_CLEARANCE - dist) * (1 - Math.abs(k) / 9);
      out[i][0] += (dx / dist) * push;
      out[i][1] += (dy / dist) * push;
    }
  }
  return { points: out, speeds };
}

/**
 * navigator.raceline/navigator.followerを、コーン回避済みのfollowerに
 * 差し替える。navigator.state === RACINGへの遷移直後に1回だけ呼ぶ。
 * followerもracelineもLaneNavigatorの公開プロパティ (private化されて
 * いない) なので、外部から代入するだけでよい。
 * @param {import('./lane_navigator.js').LaneNavigator} navigator
 * @param {Array<{x:number, y:number}>} cones
 * @param {object} trackerParams navigator.p.tracker
 */
export function applyRacelineDeflection(navigator, cones, trackerParams) {
  const { points, speeds } = deflectRacelineAroundCones(navigator.raceline.points, navigator.raceline.speed, cones);
  navigator.follower = new RacelineFollower(points, speeds, trackerParams);
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
