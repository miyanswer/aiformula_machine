// "理想検出" detector mode: instead of running UFLD on the rendered camera
// image, observes the course's actual white lines (js/course_lines.js,
// extracted from the course texture) from the vehicle's true pose, with
// UFLD-like imperfections: measurement noise growing with distance, random
// per-frame line dropouts, and no line where the course has none (dashes,
// junctions). Output is the same list of base_link LineFits the UFLD path
// produces, so everything downstream (LineTracker -> LaneNavigator) is
// identical -- this isolates "does the driving method work" from "does UFLD
// generalize to this simulator's rendering".

import { COURSE_LINES } from './course_lines.js';
import { fitLine } from './lane_navigator.js';

export const IDEAL_DETECTOR_PARAMS = {
  xMin: 1.2, // [m] closest visible ground point (camera FOV)
  xMax: 12.0, // [m]
  halfFovTan: Math.tan((51.5 * Math.PI) / 180), // horizontal half FOV of the 16:9, 70.6deg-vFOV camera
  noiseBase: 0.03, // [m] lateral noise sigma at the vehicle
  noisePerMeter: 0.01, // [m/m] extra sigma per meter of distance
  dropProb: 0.15, // per-line, per-frame dropout probability
  maxPointsPerLine: 20,
};

function gaussian() {
  let u = 0;
  let v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

export class IdealLaneDetector {
  constructor(params = IDEAL_DETECTOR_PARAMS) {
    this.p = { ...params };
  }

  /**
   * @param {{x:number,y:number,yaw:number}} pose true vehicle pose (base_link)
   * @returns {{fits: LineFit[], points: Array<{x:number[], y:number[]}>}}
   */
  detect(pose) {
    const p = this.p;
    const c = Math.cos(pose.yaw);
    const s = Math.sin(pose.yaw);
    const fits = [];
    const points = [];
    for (const name of ['outer', 'center', 'inner']) {
      if (Math.random() < p.dropProb) continue;
      const xs = [];
      const ys = [];
      for (const q of COURSE_LINES[name]) {
        if (!q) continue;
        const dx = q[0] - pose.x;
        const dy = q[1] - pose.y;
        const vx = c * dx + s * dy;
        const vy = -s * dx + c * dy;
        if (vx < p.xMin || vx > p.xMax || Math.abs(vy) > vx * p.halfFovTan) continue;
        xs.push(vx);
        ys.push(vy);
      }
      if (xs.length < 5) continue;
      const step = Math.max(1, Math.floor(xs.length / p.maxPointsPerLine));
      const sx = [];
      const sy = [];
      for (let i = 0; i < xs.length; i += step) {
        sx.push(xs[i]);
        sy.push(ys[i] + gaussian() * (p.noiseBase + p.noisePerMeter * xs[i]));
      }
      const fit = fitLine(sx, sy);
      if (!fit) continue;
      fits.push(fit);
      points.push({ x: sx, y: sy });
    }
    // Slot order carries no meaning (same as UFLD's slots after role tracking).
    for (let i = fits.length - 1; i > 0; i--) {
      const j = Math.floor(Math.random() * (i + 1));
      [fits[i], fits[j]] = [fits[j], fits[i]];
      [points[i], points[j]] = [points[j], points[i]];
    }
    return { fits, points };
  }
}
