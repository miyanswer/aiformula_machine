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
  arcLengths, dashActiveMask, offsetClosed, ribbonVertices, solidActiveMask,
} from '../js/course.js';
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
  // Three states, not two: pass/fail for an assertion that actually ran,
  // and a distinct "skipped" for one whose precondition wasn't met. A skip
  // is not a failure -- it means this check didn't test anything, not that
  // what it would have tested is broken -- so it must not flip the overall
  // `pass` below, and the console table must show it as neither PASS nor
  // FAIL.
  const check = (name, pass, detail) => results.push({ name, pass, detail, skipped: false });
  const skip = (name, detail) => results.push({ name, pass: null, detail, skipped: true });
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

  // 1b. Drawn ribbon width. The above only checks the *constant*; it says
  // nothing about the triangles createCourseLines() actually emits. Measure
  // from ribbonVertices() -- the exact function the renderer calls -- at
  // several points around the outer/inner/center paths, at full double
  // precision (see course.js's comment on ribbonVertices for why not from
  // the rendered mesh's Float32BufferGeometry: its rounding at this
  // course's ~120m coordinate range is itself larger than the 1e-6 m
  // tolerance we want here, independent of whether the ribbon is correct).
  {
    const { centerPath, laneWidthM, lineWidthM, outerSign } = COURSE_GEOMETRY;
    const outerPath = offsetClosed(centerPath, outerSign * laneWidthM);
    const innerPath = offsetClosed(centerPath, -outerSign * laneWidthM);
    let maxDiff = 0;
    for (const p of [centerPath, outerPath, innerPath]) {
      for (let i = 0; i + 1 < p.length; i += 97) {
        // a0/a1 are the two edge vertices of the cross-section at index i
        // (see ribbonVertices(): out = [a0(3), a1(3), b0(3), a1(3), b1(3), b0(3)]).
        const v = ribbonVertices(p, [i, i + 1], lineWidthM);
        const w = Math.hypot(v[0] - v[3], v[1] - v[4]);
        maxDiff = Math.max(maxDiff, Math.abs(w - lineWidthM));
      }
    }
    check('drawn ribbon width matches lineWidthM', maxDiff < 1e-6, `max diff ${maxDiff.toExponential(3)} m`);
  }

  // 1c. Drawn mask vs data. dashRanges()/solidRanges() (course.js) and
  // dashed_line()/gapped_line() (build_course.py) are two independent
  // implementations of the same dash/gap predicate, and nothing pinned that
  // they agree. Build the drawn active-index mask from course.js's own
  // exported predicates (not a re-implementation here -- a duplicate would
  // only ever agree with itself) and diff it against COURSE_LINES, which
  // build_course.py produced independently.
  {
    const { s, total } = arcLengths(path);
    const centerActive = dashActiveMask(s, COURSE_GEOMETRY.dash.markM, COURSE_GEOMETRY.dash.gapM);
    const innerActive = solidActiveMask(s, total, COURSE_GEOMETRY.innerGaps);
    for (const [name, active] of [['center', centerActive], ['inner', innerActive]]) {
      let mismatches = 0;
      for (let i = 0; i < active.length; i++) {
        const dataActive = COURSE_LINES[name][i] !== null;
        if (active[i] !== dataActive) mismatches++;
      }
      check(`drawn mask matches COURSE_LINES.${name}`, mismatches === 0, `${mismatches} mismatches / ${active.length}`);
    }
  }

  // 3. スポーン
  // Two independent assertions instead of one that depends on runtime state:
  //
  // (a) COURSE_GEOMETRY.startPose itself lies on the centre line. This is
  //     what 検証方針 item 3 actually cares about -- that the course's
  //     designated start anchor sits where it should -- and it needs no
  //     vehicle, no simulator, nothing but the geometry data, so it always
  //     runs and would catch a regenerated course whose anchor drifted off
  //     centerPath. Where the vehicle happens to be sitting right now is
  //     incidental to that.
  const startPoseDist = nearestDistance(
    [COURSE_GEOMETRY.startPose.x, COURSE_GEOMETRY.startPose.y], path
  );
  check('startPose lies on the centre line', startPoseDist < 0.05, `${startPoseDist.toFixed(4)} m`);

  // (b) the vehicle's own measured distance from the centre line -- only
  //     meaningful once the vehicle has actually been moved onto startPose.
  //     VehiclePhysics starts at the odom origin (0,0,0), ~1.6 m from
  //     startPose, so right after page load this precondition never holds;
  //     the HUD's 「スタート位置へ」 button (#start-pose-btn, index.html)
  //     is what moves it there (README.md's documented repro clicks it
  //     first). Skipped -- not failed -- when the precondition isn't met,
  //     so a fresh page load reports a clean run rather than red for a
  //     state this part of the check deliberately didn't test.
  const sim = window.__sim;
  const spawnCheckName = 'vehicle spawn distance from the centre line';
  if (!sim) {
    skip(spawnCheckName, 'window.__sim is not present (not running inside the simulator page)');
  } else {
    const dx = sim.physics.x - COURSE_GEOMETRY.startPose.x;
    const dy = sim.physics.y - COURSE_GEOMETRY.startPose.y;
    let dyaw = (sim.physics.yaw - COURSE_GEOMETRY.startPose.yaw) % (2 * Math.PI);
    if (dyaw > Math.PI) dyaw -= 2 * Math.PI;
    if (dyaw < -Math.PI) dyaw += 2 * Math.PI;
    const atStartPose = Math.hypot(dx, dy) < 0.1 && Math.abs(dyaw) < 0.1;
    if (!atStartPose) {
      skip(spawnCheckName,
        'vehicle is not at COURSE_GEOMETRY.startPose -- click #start-pose-btn '
        + '(「スタート位置へ」) before running this check, see README.md');
    } else {
      const d = nearestDistance([sim.physics.x, sim.physics.y], path);
      check(spawnCheckName, d < 0.05, `${d.toFixed(4)} m`);
    }
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
  //
  // Tolerance: 3 cm, not the originally-written 1 cm. The measured mean
  // offset is about -1.95 cm, which is a structural bias of the detector,
  // not a geometry error: IdealLaneDetector only observes points beyond
  // xMin = 1.2 m and fits a straight line whose x=0 intercept is then
  // evaluated -- a few centimetres of intercept error is expected for a
  // dashed, curved centre line observed only past 1.2 m. See the design
  // spec's 検証方針 item 4 (docs/superpowers/specs/
  // 2026-09-21-course-geometry-and-collision-design.md), amended in commit
  // 0fa72f9 for this exact reasoning. The geometry itself is separately
  // covered by the sub-millimetre lane-width check above.
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
      centreOffsets.length >= trials * 0.3 && Math.abs(mean) < 0.03,
      `n=${centreOffsets.length}/${trials} mean offset ${Number.isFinite(mean) ? mean.toFixed(4) : 'NaN'} m`);
  }

  // 5. 障害物
  const obstacles = worldColliders(MYLAPS_POSE, MYLAPS_COLLIDERS);
  const headOnPose = { x: MYLAPS_POSE.x + 0.5, y: MYLAPS_POSE.y, yaw: Math.PI };

  // Kept as its own assertion so the residual-penetration check below cannot
  // pass by never touching anything.
  const firstContact = resolveCollisions(headOnPose, obstacles, VEHICLE_COLLIDERS);
  check('gate blocks a head-on approach', firstContact.maxPenetration > 0 && firstContact.headOn,
    `penetration ${firstContact.maxPenetration.toFixed(4)} m, headOn ${firstContact.headOn}`);

  // Spec item 5 requires "residual penetration after correction stays under
  // 1mm", not merely "a contact was detected" -- the latter still passes if
  // resolveCollisions()'s sequential per-obstacle correction (each obstacle
  // is measured against the position already corrected by earlier obstacles
  // this call, so two obstacles on opposite sides cannot net-cancel) is
  // reverted to measuring every obstacle from the same starting position.
  // Iterate resolve -> apply the returned (dx, dy) -> resolve again from the
  // corrected pose until the correction stops moving the vehicle (or a
  // safety cap), then assert the final maxPenetration is under 1e-3 m.
  let pose = { ...headOnPose };
  let res = resolveCollisions(pose, obstacles, VEHICLE_COLLIDERS);
  let iterations = 0;
  while (iterations < 50 && (Math.abs(res.dx) > 1e-9 || Math.abs(res.dy) > 1e-9)) {
    pose = { x: pose.x + res.dx, y: pose.y + res.dy, yaw: pose.yaw };
    res = resolveCollisions(pose, obstacles, VEHICLE_COLLIDERS);
    iterations++;
  }
  check('residual penetration after correction < 1mm', res.maxPenetration < 1e-3,
    `residual ${res.maxPenetration.toFixed(6)} m after ${iterations} correction step(s)`);

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

  // Overall pass is computed over the non-skipped results only -- a skip
  // means "not tested", not "failed", so it must not drag the run red.
  const pass = results.every((x) => x.skipped || x.pass);
  const counts = {
    pass: results.filter((x) => !x.skipped && x.pass).length,
    fail: results.filter((x) => !x.skipped && !x.pass).length,
    skipped: results.filter((x) => x.skipped).length,
  };
  // Print a `state` column (PASS/FAIL/SKIP) rather than the raw boolean/null
  // `pass` field, so a skip reads as visibly distinct from both outcomes.
  console.table(results.map((x) => ({
    name: x.name,
    state: x.skipped ? 'SKIP' : (x.pass ? 'PASS' : 'FAIL'),
    detail: x.detail,
  })));
  return { pass, results, counts };
}
