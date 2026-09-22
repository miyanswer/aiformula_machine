// The simulator's course: models/course.glb, loaded as-is.
//
// course.glb is authored in ROS convention (ground = xy plane, height = z,
// metres): its root node carries the +90deg X rotation that cancels rosRoot's
// -90deg X, so the scene can simply be parented to rosRoot. Nothing in it is
// scaled or re-oriented here.
//
// What this module adds on top of the raw model:
//   * The course is shifted so that START_POSE (a point on the centre white
//     line) is the odom origin (0, 0) with the vehicle facing +x -- i.e. the
//     vehicle's initial pose (VehiclePhysics starts at 0, 0, 0) is the start
//     position.
//   * Everything else the simulator needs to know about the course is measured
//     from the model rather than hand-copied: the line width, the lane width, a
//     closed centre-line path (course departure), and point sets for the three
//     white lines the lane detector looks for (ideal-detector mode). Re-export
//     the GLB and these follow. START_POSE is the one hand-set value.
//
// Materials are replaced with unlit MeshBasicMaterial in the model's own base
// colours: the onboard camera image is what YOLOP/UFLD read, and it must not
// change with the sun angle.

import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

const COURSE_URL = 'models/course.glb'; // relative to index.html

// Mesh names in course.glb (see the model's outliner).
const CENTER_SOLID = 'Line_Center_Solid';
const CENTER_DASH_PREFIX = 'Line_Center_Dash';
const OUTER_BOUNDARY = 'Line_Outer_Inner'; // 2nd of the double outer line = the lane boundary
const INNER_PREFIX = 'Line_Block'; // white lines around the grass-less islands
const INNER_IGNORE_SUFFIX = '_Inner'; // 2nd line of a double pair (outside the lane)
const IGNORED_NODES = ['Cube']; // Blender's default cube, left at the origin

// White paint is a 5mm-high slab on the asphalt; only its top face is measured.
const LINE_TOP_MIN_Z = 0.004; // [m]

const PATH_STEP_M = 0.1; // spacing of the resampled centre path
const PATH_BINS = 720; // angular bins used to turn a point cloud into a loop
const INNER_BAND_M = 0.75; // island lines this close to one lane width from the centre line are the inner boundary

// Where the vehicle starts, in the course model's own frame (the frame of
// course.glb, metres / radians): on the left side of the loop, heading south --
// counter-clockwise travel. It is fitted to the centre white line there: the
// centre of Line_Center_Solid's ribbon (both edges at +/-7.5 cm) and its
// direction, from a least-squares fit of the ribbon's top-face vertices within
// 2 m of the point. Set by hand; if the model is re-exported with the loop
// moved, refit it.
const START_POSE = { x: -41.457, y: 15.109, yaw: THREE.MathUtils.degToRad(-91.42) };

// ---------------------------------------------------------------------------
// Small 2D helpers
// ---------------------------------------------------------------------------

function distToPolyline(p, poly) {
  let best = Infinity;
  for (const q of poly) {
    const d = Math.hypot(p[0] - q[0], p[1] - q[1]);
    if (d < best) best = d;
  }
  return best;
}

/** Turns an unordered point cloud around `center` into a closed polyline: one mean point per angular bin, empty bins filled by interpolation. */
function cloudToLoop(points, center) {
  const sum = new Array(PATH_BINS).fill(null);
  for (const [x, y] of points) {
    const a = Math.atan2(y - center[1], x - center[0]);
    const k = Math.min(PATH_BINS - 1, Math.floor(((a + Math.PI) / (2 * Math.PI)) * PATH_BINS));
    if (!sum[k]) sum[k] = [0, 0, 0];
    sum[k][0] += x;
    sum[k][1] += y;
    sum[k][2] += 1;
  }
  const filled = [];
  for (let k = 0; k < PATH_BINS; k++) if (sum[k]) filled.push(k);
  if (filled.length < 8) throw new Error('course: too few line points to form a loop');
  const loop = new Array(PATH_BINS);
  for (const k of filled) loop[k] = [sum[k][0] / sum[k][2], sum[k][1] / sum[k][2]];
  for (let n = 0; n < filled.length; n++) {
    const a = filled[n];
    const b = filled[(n + 1) % filled.length];
    const span = (b - a + PATH_BINS) % PATH_BINS || PATH_BINS;
    for (let s = 1; s < span; s++) {
      const t = s / span;
      loop[(a + s) % PATH_BINS] = [
        loop[a][0] + t * (loop[b % PATH_BINS][0] - loop[a][0]),
        loop[a][1] + t * (loop[b % PATH_BINS][1] - loop[a][1]),
      ];
    }
  }
  return loop;
}

/** Equal-arc-length resample of a closed polyline. */
function resampleClosed(loop, step) {
  const n = loop.length;
  const cum = [0];
  for (let i = 0; i < n; i++) {
    const a = loop[i];
    const b = loop[(i + 1) % n];
    cum.push(cum[i] + Math.hypot(b[0] - a[0], b[1] - a[1]));
  }
  const total = cum[n];
  const count = Math.max(8, Math.round(total / step));
  const out = [];
  let seg = 0;
  for (let k = 0; k < count; k++) {
    const s = (k * total) / count;
    while (cum[seg + 1] < s) seg++;
    const a = loop[seg];
    const b = loop[(seg + 1) % n];
    const t = (s - cum[seg]) / Math.max(cum[seg + 1] - cum[seg], 1e-9);
    out.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
  }
  return out;
}

function median(values) {
  const v = [...values].sort((a, b) => a - b);
  return v[Math.floor(v.length / 2)];
}

// ---------------------------------------------------------------------------
// Reading the model
// ---------------------------------------------------------------------------

/**
 * Unique top-face vertices of a line mesh, in buffer order, in `frame`
 * coordinates (metres, z up). The glTF exporter duplicates vertices per face
 * (flat normals), so positions are de-duplicated at 1mm.
 */
function topFaceVertices(mesh, frameInverse) {
  const pos = mesh.geometry.attributes.position;
  const m = new THREE.Matrix4().multiplyMatrices(frameInverse, mesh.matrixWorld);
  const v = new THREE.Vector3();
  const seen = new Set();
  const out = [];
  for (let i = 0; i < pos.count; i++) {
    v.fromBufferAttribute(pos, i).applyMatrix4(m);
    if (v.z < LINE_TOP_MIN_Z) continue;
    const key = `${Math.round(v.x * 1000)},${Math.round(v.y * 1000)}`;
    if (seen.has(key)) continue;
    seen.add(key);
    out.push([v.x, v.y]);
  }
  return out;
}

/** Line width, from the two corners of Line_Center_Solid's start cap (the first two vertices of its buffer). */
function lineWidthOf(solid) {
  if (solid.length < 2) throw new Error(`course: ${CENTER_SOLID} has too few vertices`);
  return Math.hypot(solid[0][0] - solid[1][0], solid[0][1] - solid[1][1]);
}

/**
 * Heading of a closed path (direction of travel) at the point of the path
 * nearest to (x, y), measured over +/-1.5 m of path.
 */
export function pathHeadingNear(path, x, y) {
  const n = path.length;
  let bi = 0;
  let bd = Infinity;
  path.forEach((q, i) => {
    const d = Math.hypot(q[0] - x, q[1] - y);
    if (d < bd) {
      bd = d;
      bi = i;
    }
  });
  const step = Math.max(1, Math.round(1.5 / Math.hypot(path[1][0] - path[0][0], path[1][1] - path[0][1])));
  const a = path[(bi - step + n) % n];
  const b = path[(bi + step) % n];
  return Math.atan2(b[1] - a[1], b[0] - a[0]);
}

// ---------------------------------------------------------------------------

/**
 * Loads models/course.glb into `parent` (rosRoot).
 *
 * @param {THREE.Object3D} parent expected to be rosRoot, so the model's ROS-convention coordinates need no conversion
 * @returns {Promise<{
 *   root: THREE.Group,
 *   start: {x: number, y: number, yaw: number}, // START_POSE, in the model's own frame
 *   lineWidthM: number, laneWidthM: number,
 *   centerPath: number[][],
 *   lines: {outer: number[][], center: number[][], inner: number[][]},
 * }>} all coordinates in the odom frame (origin = start of the centre line, +x = its direction)
 */
export async function loadCourse(parent) {
  const gltf = await new GLTFLoader().loadAsync(COURSE_URL);
  const scene = gltf.scene;
  for (const name of IGNORED_NODES) scene.getObjectByName(name)?.removeFromParent();

  const unlit = new Map();
  scene.traverse((obj) => {
    if (!obj.isMesh) return;
    if (!unlit.has(obj.material)) {
      unlit.set(obj.material, new THREE.MeshBasicMaterial({ color: obj.material.color, side: THREE.DoubleSide }));
    }
    obj.material = unlit.get(obj.material);
  });

  const root = new THREE.Group();
  root.name = 'course';
  root.add(scene);
  parent.add(root);
  parent.updateMatrixWorld(true);
  const frameInverse = new THREE.Matrix4().copy(parent.matrixWorld).invert();

  const meshes = [];
  scene.traverse((obj) => obj.isMesh && meshes.push(obj));
  const verts = (name) => topFaceVertices(meshes.find((m) => m.name === name), frameInverse);
  const named = (prefix, ignoreSuffix) =>
    meshes.filter((m) => m.name.startsWith(prefix) && !(ignoreSuffix && m.name.endsWith(ignoreSuffix)));
  for (const need of [CENTER_SOLID, OUTER_BOUNDARY]) {
    if (!meshes.some((m) => m.name === need)) throw new Error(`course: ${need} not found in ${COURSE_URL}`);
  }

  // --- START_POSE -> odom origin -------------------------------------------------
  const solid = verts(CENTER_SOLID);
  const start = { ...START_POSE, lineWidthM: lineWidthOf(solid) };
  const c = Math.cos(-start.yaw);
  const s = Math.sin(-start.yaw);
  // course frame -> odom frame: translate the start to (0, 0), rotate its heading to +x.
  // Every vertex below is read in the course frame (the model as authored), so
  // this is applied to the extracted points -- and only at the very end to `root`
  // itself, because topFaceVertices() reads each mesh's current matrixWorld.
  const toOdom = ([x, y]) => [c * (x - start.x) - s * (y - start.y), s * (x - start.x) + c * (y - start.y)];

  // --- the three white lines, in the odom frame -----------------------------------
  const center = [...solid, ...named(CENTER_DASH_PREFIX).flatMap((m) => topFaceVertices(m, frameInverse))].map(toOdom);
  const outer = verts(OUTER_BOUNDARY).map(toOdom);

  const xs = outer.map((p) => p[0]);
  const ys = outer.map((p) => p[1]);
  const loopCenter = [(Math.min(...xs) + Math.max(...xs)) / 2, (Math.min(...ys) + Math.max(...ys)) / 2];

  // --- centre path: closed, equally spaced, in the direction of travel starting at the origin ----
  let path = resampleClosed(cloudToLoop(center, loopCenter), PATH_STEP_M);
  let i0 = 0;
  path.forEach((p, i) => {
    if (Math.hypot(p[0], p[1]) < Math.hypot(path[i0][0], path[i0][1])) i0 = i;
  });
  path = [...path.slice(i0), ...path.slice(0, i0)];
  const heading = Math.atan2(path[5][1] - path[0][1], path[5][0] - path[0][0]);
  if (Math.cos(heading) < 0) path = [path[0], ...path.slice(1).reverse()]; // travel direction = +x at the start

  // --- lane width: centre line to the outer lane boundary, both as centre-averaged loops ----
  const outerLoop = resampleClosed(cloudToLoop(outer, loopCenter), PATH_STEP_M);
  const laneWidthM = median(path.filter((_, i) => i % 5 === 0).map((p) => distToPolyline(p, outerLoop)));

  // --- inner boundary: island lines that lie one lane from the centre line ------------------
  const inner = named(INNER_PREFIX, INNER_IGNORE_SUFFIX)
    .flatMap((m) => topFaceVertices(m, frameInverse))
    .map(toOdom)
    .filter((p) => Math.abs(distToPolyline(p, path) - laneWidthM) < INNER_BAND_M);

  // --- finally move the model itself, so the drawn course matches the odom-frame data above ----
  root.rotation.z = -start.yaw;
  root.position.set(-(c * start.x - s * start.y), -(s * start.x + c * start.y), 0);
  parent.updateMatrixWorld(true);

  return { root, start, lineWidthM: start.lineWidthM, laneWidthM, centerPath: path, lines: { outer, center, inner } };
}
