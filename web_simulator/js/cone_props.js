// web_simulator/js/cone_props.js
// コーンの3Dモデル(models/cone.glb)と当たり判定。configではなく
// 実測値を使う: models/cone.glb はルートnode "cone v1" がscale 0.001,
// 子ノード(Cone_Base/Cone_Body/Cone_Flange)がそれぞれscale 10で、
// 合成スケールは0.01 (cm -> m)。Cone_BaseのXZ範囲が±15 (実寸ではさらに
// x0.01 = ±0.15m)なので、当たり判定半径は0.15m
// (js/course_props.jsのMYLAPS_COLLIDERSのコーン半径0.15mと同じ規格の
// 450mm三角コーン)。course.jsと同じくGLTFLoaderでロードする。

import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

const MODEL_DIR = 'models/'; // relative to index.html
const CONE_URL = MODEL_DIR + 'cone.glb';

export const CONE_RADIUS = 0.15; // [m] models/cone.glb実測 (Cone_Baseの半径)
export const CONE_SCALE = 0.01; // 0.001(root) x 10(children)

/**
 * cone.glbを1回だけロードして、その場で使えるテンプレートを返す。
 * 呼び出し側 (addCone) がコーンの数だけ template.clone() する。
 * @returns {Promise<THREE.Object3D>}
 */
export async function loadConeTemplate() {
  const gltf = await new GLTFLoader().loadAsync(CONE_URL);
  return gltf.scene;
}

/**
 * コーンを1本、parent (rosRoot) に追加する。
 * @param {THREE.Object3D} parent
 * @param {THREE.Object3D} template loadConeTemplate() の戻り値
 * @param {{x:number, y:number}} pose ROS座標 (odom frame)
 * @returns {THREE.Object3D} 追加したインスタンス (削除時にparent.remove()するため返す)
 */
export function addCone(parent, template, { x, y }) {
  const instance = template.clone();
  instance.scale.setScalar(CONE_SCALE);
  instance.position.set(x, y, 0);
  parent.add(instance);
  return instance;
}

/**
 * コーンの位置配列から、collision.jsのresolveCollisions()にそのまま渡せる
 * 円コライダー配列を作る。
 * @param {Array<{x:number, y:number}>} cones
 * @returns {Array<{x:number, y:number, r:number}>}
 */
export function coneWorldColliders(cones) {
  return cones.map((c) => ({ x: c.x, y: c.y, r: CONE_RADIUS }));
}
