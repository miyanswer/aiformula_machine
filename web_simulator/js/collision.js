// 2D collision helpers for the simulator. Pure geometry -- no three.js, no
// scene access -- so the same functions back both the runtime checks in
// js/simulator.js and the verification script in tools/verify_course.js.

import { COURSE_GEOMETRY } from './course_geometry.js';

// Vehicle footprint: two circles along the body axis, in base_link metres.
// Covers roughly 1.6m x 0.8m -- the xacro body runs from the caster at
// x = -0.76 to about x = +0.7, and the 0.40 radius matches
// lane_navigator.js's RACELINE_PARAMS.vehicleHalfWidth.
export const VEHICLE_HALF_WIDTH = 0.40;
export const VEHICLE_COLLIDERS = [
  { x: 0.30, r: VEHICLE_HALF_WIDTH },
  { x: -0.50, r: VEHICLE_HALF_WIDTH },
];

/**
 * Pushes the vehicle out of any obstacle it overlaps.
 *
 * Returns the positional correction to apply, whether the contact is close
 * to head-on (the caller zeroes forward speed then -- VehiclePhysics carries
 * only a scalar forward speed, so a normal/tangential velocity split is not
 * meaningful), and the deepest penetration seen.
 *
 * @param {{x:number, y:number, yaw:number}} pose
 * @param {Array<{x:number, y:number, r:number}>} obstacles world-frame circles
 * @param {Array<{x:number, r:number}>} vehicleColliders base_link circles
 */
export function resolveCollisions(pose, obstacles, vehicleColliders = VEHICLE_COLLIDERS) {
  const c = Math.cos(pose.yaw);
  const s = Math.sin(pose.yaw);
  let dx = 0;
  let dy = 0;
  let headOn = false;
  let maxPenetration = 0;

  for (const vc of vehicleColliders) {
    // Re-evaluate the circle's world position against corrections already
    // accumulated this step, so two obstacles cannot cancel each other out.
    const wx = pose.x + c * vc.x + dx;
    const wy = pose.y + s * vc.x + dy;
    for (const o of obstacles) {
      const ox = wx - o.x;
      const oy = wy - o.y;
      const dist = Math.hypot(ox, oy);
      const penetration = vc.r + o.r - dist;
      if (penetration <= 0) continue;
      const nx = dist > 1e-9 ? ox / dist : 1;
      const ny = dist > 1e-9 ? oy / dist : 0;
      dx += nx * penetration;
      dy += ny * penetration;
      maxPenetration = Math.max(maxPenetration, penetration);
      if (c * nx + s * ny < -0.5) headOn = true;
    }
  }
  return { dx, dy, headOn, maxPenetration };
}

/**
 * Tracks where the vehicle is along the closed reference path and how far it
 * sits to the side of it. Searches only near the previous index (the path is
 * 0.10m-sampled, so +/-400 points is 40m of travel between frames) instead of
 * scanning all ~2470 points every frame.
 */
export class PathTracker {
  constructor(path = COURSE_GEOMETRY.centerPath, window = 400) {
    this.path = path;
    this.window = window;
    this.index = 0;
    this.s = [0];
    for (let i = 1; i < path.length; i++) {
      this.s.push(this.s[i - 1] + Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]));
    }
  }

  reset(index = 0) {
    this.index = index;
  }

  /** @returns {{offset:number, s:number, index:number}} offset is +left of the path */
  update(x, y) {
    const n = this.path.length;
    let best = Infinity;
    let bi = this.index;
    for (let k = -this.window; k <= this.window; k++) {
      const i = ((this.index + k) % n + n) % n;
      const d = (this.path[i][0] - x) ** 2 + (this.path[i][1] - y) ** 2;
      if (d < best) {
        best = d;
        bi = i;
      }
    }
    this.index = bi;
    const a = this.path[bi];
    const b = this.path[(bi + 1) % n];
    const tx = b[0] - a[0];
    const ty = b[1] - a[1];
    const len = Math.hypot(tx, ty) || 1e-12;
    const offset = (-(ty / len)) * (x - a[0]) + (tx / len) * (y - a[1]);
    return { offset, s: this.s[bi], index: bi };
  }
}

/**
 * Counts course departures with hysteresis: one count when the vehicle first
 * crosses `leaveM`, and no further counts until it comes back inside
 * `returnM`. Driving is never blocked -- this only reports.
 */
export class DepartureMonitor {
  constructor({ leaveM, returnM } = {}) {
    const lane = COURSE_GEOMETRY.laneWidthM;
    const line = COURSE_GEOMETRY.lineWidthM;
    // Any part of the vehicle past the outer edge of the boundary line.
    this.leaveM = leaveM !== undefined ? leaveM : lane + line / 2 - VEHICLE_HALF_WIDTH;
    this.returnM = returnM !== undefined ? returnM : this.leaveM - 0.175;
    this.count = 0;
    this.outside = false;
  }

  reset() {
    this.count = 0;
    this.outside = false;
  }

  /** @returns {{outside:boolean, count:number, justLeft:boolean}} */
  update(offset) {
    const magnitude = Math.abs(offset);
    let justLeft = false;
    if (!this.outside && magnitude > this.leaveM) {
      this.outside = true;
      this.count += 1;
      justLeft = true;
    } else if (this.outside && magnitude < this.returnM) {
      this.outside = false;
    }
    return { outside: this.outside, count: this.count, justLeft };
  }
}
