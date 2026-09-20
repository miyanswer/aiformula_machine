// Browser-side verification for the generated course and the collision
// helpers. There is no node on this host, so this is run by importing it in
// the page and calling runChecks():
//
//   const m = await import('/web_simulator/tools/verify_course.js');
//   await m.runChecks();
//
// Checks 1-6 of the design doc's 検証方針 section. Checks 7-8 (autonomous
// laps) are driven by hand, not from here.

import { COURSE_GEOMETRY } from '../js/course_geometry.js';
import { COURSE_LINES } from '../js/course_lines.js';
import {
  DepartureMonitor, VEHICLE_COLLIDERS, resolveCollisions,
} from '../js/collision.js';
import { MYLAPS_COLLIDERS, MYLAPS_POSE, worldColliders } from '../js/course_props.js';
import { IdealLaneDetector } from '../js/ideal_lane_detector.js';

function nearestDistance(point, path) {
  let best = Infinity;
  for (let i = 0; i < path.length; i++) {
    const a = path[i];
    const b = path[(i + 1) % path.length];
    const abx = b[0] - a[0];
    const aby = b[1] - a[1];
    const l2 = Math.max(abx * abx + aby * aby, 1e-12);
    const t = Math.max(0, Math.min(1, ((point[0] - a[0]) * abx + (point[1] - a[1]) * aby) / l2));
    const d = Math.hypot(point[0] - a[0] - t * abx, point[1] - a[1] - t * aby);
    if (d < best) best = d;
  }
  return best;
}

export async function runChecks() {
  const results = [];
  const check = (name, pass, detail) => results.push({ name, pass, detail });
  const path = COURSE_GEOMETRY.centerPath;

  // 1. 寸法
  // COURSE_LINES.outer/inner may contain null entries (the inner boundary's
  // junction openings) -- skip them rather than measuring a gap. center is
  // not walked here: it is the reference path itself, not a boundary to
  // measure distance from.
  for (const side of ['outer', 'inner']) {
    let min = Infinity;
    let max = -Infinity;
    for (const p of COURSE_LINES[side]) {
      if (!p) continue;
      const d = nearestDistance(p, path);
      min = Math.min(min, d);
      max = Math.max(max, d);
    }
    check(`lane width (${side})`,
      Math.abs(min - COURSE_GEOMETRY.laneWidthM) < 0.001 && Math.abs(max - COURSE_GEOMETRY.laneWidthM) < 0.001,
      `min ${min.toFixed(4)} max ${max.toFixed(4)}`);
  }
  check('line width constant', COURSE_GEOMETRY.lineWidthM === 0.15, `${COURSE_GEOMETRY.lineWidthM}`);

  // 3. スポーン
  const sim = window.__sim;
  if (sim) {
    const d = nearestDistance([sim.physics.x, sim.physics.y], path);
    check('spawn on the centre line', d < 0.05, `${d.toFixed(4)} m`);
  }

  // 4. 理想検出との一致
  // IdealLaneDetector (js/ideal_lane_detector.js) adds per-point Gaussian
  // lateral noise (sigma = noiseBase + noisePerMeter * distance, i.e. ~0.03m
  // near the vehicle) and drops each of the outer/center/inner lines with
  // probability dropProb = 0.15 per call, so a single detect() call is not
  // reliable enough to assert on by itself. Instead, call it repeatedly from
  // the exact start pose (which sits on the centre line by construction, so
  // the centre line's fit should have a lateral intercept at x=0 -- the
  // base_link origin -- very close to 0) and average the result: the noise
  // is zero-mean, so averaging over enough trials makes the random draws
  // cancel out and converges on the true offset regardless of any single
  // trial. Fits are returned in random slot order with no role label, so the
  // "centre line" fit is identified per-trial as whichever returned fit has
  // the smallest |yAt(0)| -- the outer/inner fits sit roughly +/-3.5m away
  // at this pose, so they are not mistaken for the centre line.
  {
    const detector = new IdealLaneDetector();
    const pose = {
      x: COURSE_GEOMETRY.startPose.x,
      y: COURSE_GEOMETRY.startPose.y,
      yaw: COURSE_GEOMETRY.startPose.yaw,
    };
    const trials = 40;
    const centreOffsets = [];
    for (let i = 0; i < trials; i++) {
      const { fits } = detector.detect(pose);
      if (fits.length === 0) continue;
      let best = fits[0];
      for (const f of fits) if (Math.abs(f.yAt(0)) < Math.abs(best.yAt(0))) best = f;
      if (Math.abs(best.yAt(0)) < 1.0) centreOffsets.push(best.yAt(0));
    }
    const mean = centreOffsets.length
      ? centreOffsets.reduce((a, b) => a + b, 0) / centreOffsets.length
      : NaN;
    check('ideal detector frame matches geometry',
      centreOffsets.length >= trials * 0.3 && Math.abs(mean) < 0.05,
      `n=${centreOffsets.length}/${trials} mean offset ${Number.isFinite(mean) ? mean.toFixed(4) : 'NaN'} m`);
  }

  // 5. 障害物
  const obstacles = worldColliders(MYLAPS_POSE, MYLAPS_COLLIDERS);
  const headOnPose = { x: MYLAPS_POSE.x + 0.5, y: MYLAPS_POSE.y, yaw: Math.PI };
  const r = resolveCollisions(headOnPose, obstacles, VEHICLE_COLLIDERS);
  check('gate blocks a head-on approach', r.maxPenetration > 0 && r.headOn,
    `penetration ${r.maxPenetration.toFixed(4)} m, headOn ${r.headOn}`);
  const clearPose = { x: MYLAPS_POSE.x + 20, y: MYLAPS_POSE.y, yaw: Math.PI };
  const clear = resolveCollisions(clearPose, obstacles, VEHICLE_COLLIDERS);
  check('no contact when clear', clear.maxPenetration === 0, `penetration ${clear.maxPenetration}`);

  // 6. 逸脱判定
  const monitor = new DepartureMonitor();
  const seen = [];
  for (const offset of [0, 2.0, 3.0, 3.1, 3.2, 3.3, 2.9, 3.2]) {
    seen.push(monitor.update(offset).count);
  }
  check('departure counts once per excursion', JSON.stringify(seen) === JSON.stringify([0, 0, 0, 0, 1, 1, 1, 2]),
    JSON.stringify(seen));

  const pass = results.every((x) => x.pass);
  console.table(results);
  return { pass, results };
}
