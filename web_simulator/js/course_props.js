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
    group.add(new THREE.Mesh(geometry, new THREE.MeshLambertMaterial({ color })));
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

// Placement: on the course centre line (js/course_lines.js "center"), 25m past
// the exit of the second corner, measured along the centre line from the
// vehicle's spawn point (SIM_START_POSE) in the driving direction.
//
//   corner 2 (the long left-hander onto the top straight) exits at s=111.0m,
//   world (27.66, 68.87); +25m along the centre line lands on s=136.0m.
//
// yaw: the arch spans the track and its LED panel (model +y) faces oncoming
// traffic. The top straight is driven in -x, so model +y must point to world
// +x, i.e. yaw = track heading + 90deg = 180 + 90 = -90deg.
//
// z is lifted just clear of the course texture plane (COURSE_POSE.z = 0.01)
// so the base bars don't z-fight with it.
export const MYLAPS_POSE = { x: 2.928, y: 68.442, z: 0.02, roll: 0, pitch: 0, yaw: -Math.PI / 2 };

/**
 * Loads models/MyLaps.obj and adds it to `parent` (expected to be rosRoot, so
 * the pose above is plain ROS x/y/z + rpy).
 *
 * @param {THREE.Object3D} parent
 * @param {(object3d: THREE.Object3D, pose: object) => void} setPose simulator.js's URDF-convention pose helper
 */
export function addMyLapsGantry(parent, setPose) {
  loadObj(MODEL_DIR + 'MyLaps.obj', MODEL_DIR + 'MyLaps.mtl')
    .then((gantry) => {
      gantry.scale.setScalar(MYLAPS_SCALE);
      const root = new THREE.Group();
      setPose(root, MYLAPS_POSE);
      root.add(gantry);
      parent.add(root);
    })
    .catch((err) => console.error('Failed to load models/MyLaps.obj', err));
}
