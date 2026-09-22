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
// cone.glb自身が root=0.001 と children=10 を持ち、合成済みで cm -> m
// (0.01) になっている。ここでさらに 0.01 を掛けると 1/100 の極小サイズに
// なり、地面に埋もれて透明に見えてしまう。
export const CONE_SCALE = 1;

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
  const instance = template.clone(true);
  instance.scale.setScalar(CONE_SCALE);
  // glTF は Y-up、rosRoot 配下のモデル座標は ROS と同じ Z-up。Y軸をZ軸へ
  // 向けてから rosRoot の座標変換に渡すことで、コーンを路面に対して直立させる。
  instance.rotateX(Math.PI / 2);
  // 路面(z=0)に沈み込んで透明化・不可視化するのを防ぐため、5mm浮かせ、描画順を保証
  instance.position.set(x, y, 0.005);
  instance.traverse((child) => {
    if (child.isMesh) {
      child.castShadow = true;
      child.receiveShadow = true;
      if (child.material) {
        child.material = child.material.clone();
        child.material.transparent = false;
        child.material.opacity = 1.0;
        child.material.depthWrite = true;
        child.material.depthTest = true;
      }
    }
  });
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
