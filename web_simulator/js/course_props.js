// Static course props (scenery only -- no collision, no lap/gate logic, same
// as the course texture itself). Currently just the MyLaps timing gantry the
// user modelled (models/MyLaps.obj + .mtl).
//
// Why a parser here instead of three/addons/loaders/OBJLoader.js: vendor/ is a
// hand-picked offline copy of three.js (see README "オフライン環境でも動くよう
// vendor/ 配下にローカル同梱"), and it only carries ColladaLoader/TGALoader.
// MyLaps.obj is about as plain as Wavefront gets -- v/vt/vn, all-triangle
// `f a/b/c` faces, 9 `usemtl` runs, and an .mtl with nothing but `Kd` -- so a
// ~60-line parser covers it without adding two more vendored addon files.

import * as THREE from 'three';
import { pathHeadingNear } from './course.js';

const MODEL_DIR = 'models/'; // relative to index.html

// ---------------------------------------------------------------------------
// Minimal Wavefront OBJ/MTL reader
// ---------------------------------------------------------------------------

/** @returns {Map<string, THREE.Color>} material name -> diffuse color */
function parseMtl(text) {
  const materials = new Map();
  let current = null;
  for (const line of text.split('\n')) {
    const parts = line.trim().split(/\s+/);
    if (parts[0] === 'newmtl') {
      current = parts.slice(1).join(' ');
      materials.set(current, new THREE.Color(0xcccccc));
    } else if (parts[0] === 'Kd' && current !== null) {
      // .mtl Kd is sRGB, but three's working color space is srgb-linear and
      // setRGB() defaults to it -- feeding sRGB numbers in raw skips the
      // decode and washes every material out (0.098 black renders as #585858
      // grey, the 0.91/0.29/0.12 cone orange as #f59360 salmon). Naming the
      // source color space makes setRGB decode it, same as three's own
      // MTLLoader does with convertSRGBToLinear().
      materials.get(current).setRGB(+parts[1], +parts[2], +parts[3], THREE.SRGBColorSpace);
    }
  }
  return materials;
}

/**
 * One Mesh per `usemtl` run (9 for MyLaps.obj), non-indexed. Faces are assumed
 * to be triangles -- MyLaps.obj is 6746/6746 triangles, and anything else is
 * skipped rather than silently mis-triangulated.
 *
 * @param {string} text raw .obj contents
 * @param {Map<string, THREE.Color>} materials from parseMtl
 * @returns {THREE.Group}
 */
function parseObj(text, materials) {
  const v = [];
  const vn = [];
  const runs = new Map(); // material name -> {position: number[], normal: number[]}
  let current = '';

  const runFor = (name) => {
    if (!runs.has(name)) runs.set(name, { position: [], normal: [] });
    return runs.get(name);
  };

  for (const line of text.split('\n')) {
    const parts = line.trim().split(/\s+/);
    switch (parts[0]) {
      case 'v':
        v.push([+parts[1], +parts[2], +parts[3]]);
        break;
      case 'vn':
        vn.push([+parts[1], +parts[2], +parts[3]]);
        break;
      case 'usemtl':
        current = parts.slice(1).join(' ');
        break;
      case 'f': {
        if (parts.length !== 4) break; // non-triangle: see note above
        const run = runFor(current);
        for (let i = 1; i <= 3; i++) {
          // "v/vt/vn", 1-based; vt is unused (the .mtl has no texture maps).
          const [vi, , ni] = parts[i].split('/');
          run.position.push(...v[+vi - 1]);
          if (ni) run.normal.push(...vn[+ni - 1]);
        }
        break;
      }
      default:
        break;
    }
  }

  const group = new THREE.Group();
  for (const [name, run] of runs) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(run.position, 3));
    if (run.normal.length === run.position.length) {
      geometry.setAttribute('normal', new THREE.Float32BufferAttribute(run.normal, 3));
    } else {
      geometry.computeVertexNormals();
    }
    // Lambert (not Standard/Phong): matches how the vehicle meshes are styled
    // -- lit, but with no specular highlight to blow out under the sun light.
    const color = materials.get(name) ?? new THREE.Color(0xcccccc);
    const mesh = new THREE.Mesh(geometry, new THREE.MeshLambertMaterial({ color }));
    mesh.name = name; // .mtl の材質名 (信号の LED 面を setSignalLight() で探す)
    group.add(mesh);
  }
  return group;
}

async function loadObj(objUrl, mtlUrl) {
  const [objText, mtlText] = await Promise.all(
    [objUrl, mtlUrl].map(async (url) => {
      const res = await fetch(url);
      if (!res.ok) throw new Error(`${url}: HTTP ${res.status}`);
      return res.text();
    })
  );
  return parseObj(objText, parseMtl(mtlText));
}

// ---------------------------------------------------------------------------
// MyLaps timing gantry
// ---------------------------------------------------------------------------

// MyLaps.obj is modelled Z-up with its base on z=0 (same convention as ROS),
// so it needs no axis fix-up -- unlike the .dae vehicle meshes, nothing here
// pre-rotates it. Its units are centimetres: the three cones are 30x30cm at
// the base and 45cm tall, i.e. real 450mm traffic cones, which puts the arch
// at 1.13m tall with 0.46m between the post centres.
const MYLAPS_SCALE = 0.01; // cm -> m

// Placement: the gantry (the timing signal) stands at MYLAPS_POSITION, given in
// the odom frame -- the frame whose origin is the vehicle's start (see START_POSE
// in course.js), so it moves with the course if that start changes. The point
// is on the dashed centre line, 5cm off the centre path, on a straight
// heading west (about 150m into the lap).
export const MYLAPS_POSITION = { x: 0.65, y: 78.6 }; // [m]

// Height of the base bars' underside. The white paint is a 5mm slab on the
// asphalt, so lift the gantry just above it instead of burying the bars in it.
const MYLAPS_Z = 0.005; // [m]

/**
 * Pose of the gantry at MYLAPS_POSITION. The arch spans the track and its LED
 * panel (model +y) faces oncoming traffic, so yaw = the track's direction of
 * travel there + 90deg; the gantry is squared to the track rather than to the
 * world axes.
 *
 * @param {number[][]} centerPath course.js's closed centre-line path (odom frame)
 */
export function mylapsPoseOnPath(centerPath) {
  const heading = pathHeadingNear(centerPath, MYLAPS_POSITION.x, MYLAPS_POSITION.y);
  return { x: MYLAPS_POSITION.x, y: MYLAPS_POSITION.y, z: MYLAPS_Z, roll: 0, pitch: 0, yaw: heading + Math.PI / 2 };
}

// Collision footprint, as circles in the model's own XY plane (metres,
// relative to the gantry's pose). Listed explicitly rather than derived from the
// .obj's group bounding boxes, so swapping the model out cannot silently
// change what the vehicle can hit. Values are the group extents of
// MyLaps.obj scaled by MYLAPS_SCALE:
//   Post_L/R   x = -/+23cm, y = -3.2cm, radius 1.9cm
//   Cone1/2/3  x = -31 / 0 / +31cm, y = 55cm, base radius 15cm
export const MYLAPS_COLLIDERS = [
  { x: -0.23, y: -0.032, r: 0.019 },
  { x: 0.23, y: -0.032, r: 0.019 },
  { x: -0.31, y: 0.55, r: 0.15 },
  { x: 0.0, y: 0.55, r: 0.15 },
  { x: 0.31, y: 0.55, r: 0.15 },
];

/**
 * Model-local collider circles placed into the world by a ROS pose.
 * @param {{x:number, y:number, yaw:number}} pose
 * @param {Array<{x:number, y:number, r:number}>} colliders
 * @returns {Array<{x:number, y:number, r:number}>}
 */
export function worldColliders(pose, colliders) {
  const c = Math.cos(pose.yaw);
  const s = Math.sin(pose.yaw);
  return colliders.map((o) => ({
    x: pose.x + c * o.x - s * o.y,
    y: pose.y + s * o.x + c * o.y,
    r: o.r,
  }));
}

/**
 * Loads models/MyLaps.obj and adds it to `parent` (expected to be rosRoot, so
 * `pose` is plain ROS x/y/z + rpy).
 *
 * @param {THREE.Object3D} parent
 * @param {(object3d: THREE.Object3D, pose: object) => void} setPose simulator.js's URDF-convention pose helper
 * @param {{x:number, y:number, z:number, roll:number, pitch:number, yaw:number}} pose from mylapsPoseOnPath()
 * @returns {THREE.Group} the root group, positioned with the given pose
 */
export function addMyLapsGantry(parent, setPose, pose) {
  const root = new THREE.Group();
  setPose(root, pose);
  parent.add(root);
  loadObj(MODEL_DIR + 'MyLaps.obj', MODEL_DIR + 'MyLaps.mtl')
    .then((gantry) => {
      gantry.scale.setScalar(MYLAPS_SCALE);
      root.add(gantry);
      if (root.userData.signal) setSignalLight(root, root.userData.signal);
    })
    .catch((err) => console.error('Failed to load models/MyLaps.obj', err));
  return root;
}

// ---------------------------------------------------------------------------
// 信号 (MyLaps パネルの LED 面)
// ---------------------------------------------------------------------------

// LED 面の色。赤は MyLaps.mtl の "LED_赤" の Kd そのまま。緑は traffic_light.pt が
// traffic_light_green と判定する程度の明るい緑。
const SIGNAL_COLORS = {
  red: [0.894118, 0.164706, 0.215686],
  green: [0.1, 0.85, 0.35],
};

/**
 * MyLaps パネルの LED 面 (材質 "LED_赤" の Mesh) の色を変える。モデルの読込前に呼ばれたら
 * 読込後に反映する (root.userData.signal に覚えておく)。
 * @param {THREE.Object3D} root addMyLapsGantry() の戻り値
 * @param {'red'|'green'} color
 */
export function setSignalLight(root, color) {
  root.userData.signal = color;
  const rgb = SIGNAL_COLORS[color];
  root.traverse((o) => {
    if (o.isMesh && o.name.startsWith('LED')) {
      o.material.color.setRGB(rgb[0], rgb[1], rgb[2], THREE.SRGBColorSpace);
      // 自発光を少し足して, 照明の向きによらず信号らしく見せる
      o.material.emissive.setRGB(rgb[0] * 0.35, rgb[1] * 0.35, rgb[2] * 0.35, THREE.SRGBColorSpace);
    }
  });
}
