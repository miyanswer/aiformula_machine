import * as THREE from 'three';
import { COURSE_GEOMETRY } from './course_geometry.js';

// Ground texture: the user's own course layout image with the outer loop's
// three white lines erased (web_simulator/tools/build_course.py). Those three
// lines are drawn as geometry instead -- see createCourseLines() -- so that
// the 15cm line width and 3.5m lane width are exact rather than limited by
// the texture's 10.4cm/px resolution. Everything else in the image (the
// inner roads, crossings, parking bays and grass islands) is still the
// texture, scaled so its outer loop lands on the generated geometry.
const COURSE_IMAGE_URL = 'png/shihou_cource_base.png'; // relative to index.html
const COURSE_IMAGE_ASPECT = 1024 / 819; // width / height, from the source PNG

// 100m -> 106.80m: the traced lane width averaged 3.2772m, so scaling by
// 3.5 / 3.2772 = 1.0680 puts the texture's own loop on top of the generated
// 3.5m lanes. See tools/build_course.py SCALE_K.
export const COURSE_WIDTH_M = 106.80;
export const COURSE_DEPTH_M = COURSE_WIDTH_M / COURSE_IMAGE_ASPECT;

const LINE_COLOR = 0xdbd4dd; // the source PNG's own white-line colour

const textureLoader = new THREE.TextureLoader();

export function createCourseTexture() {
  const texture = textureLoader.load(COURSE_IMAGE_URL, undefined, undefined, (err) =>
    console.error(`Failed to load course image ${COURSE_IMAGE_URL}`, err)
  );
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = 4;
  return texture;
}

// --- White lines, generated from COURSE_GEOMETRY.centerPath ---------------

// Unit left-hand normal at each point of a closed polyline (central
// difference, so it matches the tangent the build tool used).
function normalsClosed(path) {
  const n = path.length;
  return path.map((_, i) => {
    const a = path[(i - 1 + n) % n];
    const b = path[(i + 1) % n];
    const tx = b[0] - a[0];
    const ty = b[1] - a[1];
    const len = Math.hypot(tx, ty) || 1e-12;
    return [-ty / len, tx / len];
  });
}

// Exported so tools/verify_course.js can offset centerPath the same way
// createCourseLines() does (to rebuild outerPath/innerPath) when measuring
// the drawn ribbon width -- reusing this instead of re-deriving the offset
// keeps the check tied to the actual generator, not a parallel copy of it.
export function offsetClosed(path, distance) {
  const normals = normalsClosed(path);
  return path.map((p, i) => [p[0] + normals[i][0] * distance, p[1] + normals[i][1] * distance]);
}

// Cumulative arc length of a closed polyline, plus its total. Exported so
// tools/verify_course.js can compute the same `s`/`total` it needs to call
// solidActiveMask()/dashActiveMask() below.
export function arcLengths(path) {
  const s = [0];
  for (let i = 1; i < path.length; i++) {
    s.push(s[i - 1] + Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]));
  }
  const total = s[s.length - 1] + Math.hypot(
    path[0][0] - path[path.length - 1][0], path[0][1] - path[path.length - 1][1]
  );
  return { s, total };
}

// Builds one flat ribbon of the given width along `path[from..to]`, as a
// triangle list in the XY plane at z = 0 (the caller lifts the whole group).
// Exported so tools/verify_course.js can measure the drawn ribbon width from
// this function's own (full double-precision) output, rather than from the
// rendered mesh's BufferGeometry -- that stores vertices in a Float32Array,
// whose rounding at this course's ~120m coordinate range is large enough on
// its own (~1e-6..1e-5 m) to swamp a sub-micrometre tolerance, regardless of
// whether the ribbon math itself is correct.
export function ribbonVertices(path, indices, width) {
  const half = width / 2;
  const normals = normalsClosed(path);
  const out = [];
  for (let k = 0; k + 1 < indices.length; k++) {
    const i = indices[k];
    const j = indices[k + 1];
    const [ax, ay] = path[i];
    const [bx, by] = path[j];
    const [anx, any] = normals[i];
    const [bnx, bny] = normals[j];
    const a0 = [ax + anx * half, ay + any * half, 0];
    const a1 = [ax - anx * half, ay - any * half, 0];
    const b0 = [bx + bnx * half, by + bny * half, 0];
    const b1 = [bx - bnx * half, by - bny * half, 0];
    out.push(...a0, ...a1, ...b0);
    out.push(...a1, ...b1, ...b0);
  }
  return out;
}

// centerPath is a closed ring (index 0 follows index length-1), so a run of
// "active" points can wrap past that seam -- a solid stretch or a dash mark
// straddling index 0 is one run, not two, and the ring's own last->first
// segment must be drawn when both its endpoints are active. Groups a
// per-point boolean into such circular runs; each returned array is the
// index chain to hand to ribbonVertices, already in wrap-correct order.
function circularRuns(active) {
  const n = active.length;
  if (active.every(Boolean)) {
    // Nothing skipped anywhere: one run covering the whole ring, explicitly
    // closed back to index 0 so the seam segment (length-1 -> 0) is drawn.
    const full = active.map((_, i) => i);
    full.push(0);
    return [full];
  }
  // At least one inactive point exists, so it's safe to start the scan
  // there: every run is then bounded by inactive points on both sides
  // within this single pass, with no run left straddling the array's own
  // start/end (that boundary is now inside an inactive stretch instead).
  const start = active.findIndex((a) => !a);
  const runs = [];
  let current = [];
  for (let k = 0; k < n; k++) {
    const i = (start + k) % n;
    if (active[i]) {
      current.push(i);
    } else {
      if (current.length > 1) runs.push(current);
      current = [];
    }
  }
  if (current.length > 1) runs.push(current);
  return runs;
}

// Per-point "is this index drawn" predicates, factored out of
// solidRanges()/dashRanges() below and exported so tools/verify_course.js
// can build the same active-index mask createCourseLines() actually draws
// and diff it against COURSE_LINES -- instead of re-implementing the
// skip/dash predicate in the check, which would only ever agree with
// itself and prove nothing about whether course_lines.js matches.
export function solidActiveMask(s, total, skip) {
  const inSkip = (value) => skip.some(([s0, s1]) => {
    const v0 = value;
    const v1 = value + total;
    return (v0 >= s0 && v0 < s1) || (v1 >= s0 && v1 < s1);
  });
  return s.map((value) => !inSkip(value));
}

export function dashActiveMask(s, markM, gapM) {
  const pitch = markM + gapM;
  return s.map((value) => value % pitch < markM);
}

// Index ranges to draw, given arc-length intervals to skip (junction
// openings) or a dash pattern.
function solidRanges(s, total, skip) {
  return circularRuns(solidActiveMask(s, total, skip));
}

function dashRanges(s, markM, gapM) {
  return circularRuns(dashActiveMask(s, markM, gapM));
}

function ribbonMesh(path, ranges, width, material) {
  const vertices = [];
  ranges.forEach((indices) => vertices.push(...ribbonVertices(path, indices, width)));
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
  return new THREE.Mesh(geometry, material);
}

/**
 * The outer loop's three white lines, generated from the reference path so
 * they are exactly COURSE_GEOMETRY.lineWidthM wide and exactly
 * COURSE_GEOMETRY.laneWidthM apart. Returns a Group in ROS coordinates,
 * lifted just clear of the course texture plane (z = 0.02).
 *
 * MeshBasicMaterial (unlit), like the texture plane itself, so the lines
 * read the same under any lighting -- what the onboard camera feeds to
 * YOLOP/UFLD must not depend on the sun angle.
 */
export function createCourseLines() {
  const { centerPath, laneWidthM, lineWidthM, dash, innerGaps, outerSign } = COURSE_GEOMETRY;
  const { s, total } = arcLengths(centerPath);
  const material = new THREE.MeshBasicMaterial({ color: LINE_COLOR, side: THREE.DoubleSide });

  // Which side of centerPath is "outer" is a decision the generator already
  // made (build_course.py's sign_outer, exported here as outerSign); this
  // renderer just follows it rather than guessing, so the two stay in sync
  // by construction instead of by a sign someone has to eyeball and fix.
  const outerPath = offsetClosed(centerPath, outerSign * laneWidthM);
  const innerPath = offsetClosed(centerPath, -outerSign * laneWidthM);

  const group = new THREE.Group();
  group.add(ribbonMesh(outerPath, solidRanges(s, total, []), lineWidthM, material));
  group.add(ribbonMesh(innerPath, solidRanges(s, total, innerGaps), lineWidthM, material));
  group.add(ribbonMesh(centerPath, dashRanges(s, dash.markM, dash.gapM), lineWidthM, material));
  group.position.z = 0.02;
  return group;
}
