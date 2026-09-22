// web_simulator/js/cone_editor.js
// コーンの自由配置エディタ。3Dビュー上でのクリック/ドラッグ/削除を扱う
// (UI入力とlocalStorage永続化のみ; 3Dモデルの生成はjs/cone_props.jsが
// simulator.js側のonChangeコールバックで行う)。
import * as THREE from 'three';

const STORAGE_KEY = 'aiformula_cones_v1';
const DRAG_THRESHOLD_PX = 5;
const PICK_RADIUS_M = 0.3; // 既存コーンのヒットテスト半径

function loadStored() {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return [];
    const arr = JSON.parse(raw);
    if (!Array.isArray(arr)) return [];
    return arr.filter((c) => c && typeof c.id === 'string' && Number.isFinite(c.x) && Number.isFinite(c.y) && Math.abs(c.x) < 100 && Math.abs(c.y) < 100);
  } catch (err) {
    console.warn('cone_editor: failed to load localStorage, starting empty', err);
    return [];
  }
}

function save(cones) {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(cones.map(({ id, x, y }) => ({ id, x, y }))));
  } catch (err) {
    console.warn('cone_editor: failed to save to localStorage', err);
  }
}

let nextId = 1;
function makeId() { return `cone_${Date.now().toString(36)}_${(nextId++).toString(36)}`; }

/**
 * @param {{raycastTarget: THREE.Object3D, rosRoot: THREE.Object3D, camera: THREE.Camera,
 *   domElement: HTMLElement, onChange: (cones: Array<{id:string,x:number,y:number}>) => void,
 *   orbitControls?: {enabled: boolean}}} opts
 */
export function createConeEditor({ raycastTarget, rosRoot, camera, domElement, onChange, orbitControls }) {
  const cones = loadStored();
  let enabled = false;
  let dragId = null;
  let downPos = null;
  let moved = false;

  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();

  function emit() { save(cones); onChange([...cones]); }

  function groundPointFromEvent(evt) {
    const rect = domElement.getBoundingClientRect();
    pointer.x = ((evt.clientX - rect.left) / rect.width) * 2 - 1;
    pointer.y = -((evt.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);
    const hits = raycaster.intersectObject(raycastTarget, true);
    if (!hits.length) return null;
    // Raycaster.intersectObject()のpointは常にTHREEワールド座標。raycastTarget
    // (js/simulator.jsの`ground`背景プレーン)はrosRootの子ではなく直接sceneに
    // 追加されているため、rosRoot.worldToLocal()でROS/odomフレームのローカル
    // 座標に変換する必要がある (rosRootはcourse.glb/MyLapsゲート/車両と同じ、
    // ROS座標をそのままローカル座標として使う親)。
    const local = rosRoot.worldToLocal(hits[0].point.clone());
    return { x: local.x, y: local.y };
  }

  function findNear(x, y) {
    return cones.find((c) => Math.hypot(c.x - x, c.y - y) < PICK_RADIUS_M) || null;
  }

  function onPointerDown(evt) {
    if (!enabled) return;
    downPos = { clientX: evt.clientX, clientY: evt.clientY };
    moved = false;
    const gp = groundPointFromEvent(evt);
    if (!gp) return;
    const hit = findNear(gp.x, gp.y);
    dragId = hit ? hit.id : null;
    if (dragId && orbitControls) {
      orbitControls.enabled = false;
    }
  }

  function onPointerMove(evt) {
    if (!enabled || !downPos) return;
    const dx = evt.clientX - downPos.clientX, dy = evt.clientY - downPos.clientY;
    if (Math.hypot(dx, dy) >= DRAG_THRESHOLD_PX) {
      moved = true; // マウス移動があったら視点回転でも moved = true にする
    }
    if (dragId === null) return; // コーンをつかんでいない場合は視点回転なのでコーン移動はしない
    const gp = groundPointFromEvent(evt);
    if (!gp) return;
    const c = cones.find((k) => k.id === dragId);
    if (c) { c.x = gp.x; c.y = gp.y; emit(); }
    evt.stopPropagation();
  }

  function onPointerUp(evt) {
    if (!enabled) return;
    const wasDragId = dragId;
    const wasMoved = moved;
    const dp = downPos;
    dragId = null; downPos = null; moved = false;
    if (orbitControls) orbitControls.enabled = true;

    // マウスが動いた場合 (視点回転やコーン移動ドラッグ) は新規コーン作成・削除を行わない
    if (dp) {
      const dist = Math.hypot(evt.clientX - dp.clientX, evt.clientY - dp.clientY);
      if (dist >= DRAG_THRESHOLD_PX || wasMoved) {
        return;
      }
    }

    if (wasDragId) {
      // 既存コーンのクリック(ドラッグなし) = 削除
      const idx = cones.findIndex((c) => c.id === wasDragId);
      if (idx >= 0) { cones.splice(idx, 1); emit(); }
      evt.stopPropagation();
      return;
    }

    // 何もない地面のクリック(ドラッグなし) = 新規コーン追加
    const gp = groundPointFromEvent(evt);
    if (!gp) return;
    // 既存コーンの至近距離なら重複配置しない
    if (findNear(gp.x, gp.y)) return;

    cones.push({ id: makeId(), x: gp.x, y: gp.y });
    emit();
    evt.stopPropagation();
  }

  domElement.addEventListener('pointerdown', onPointerDown);
  domElement.addEventListener('pointermove', onPointerMove);
  domElement.addEventListener('pointerup', onPointerUp);

  return {
    enable() { enabled = true; },
    disable() { enabled = false; dragId = null; downPos = null; },
    isEnabled() { return enabled; },
    get cones() { return [...cones]; },
    addCone(x, y) { cones.push({ id: makeId(), x, y }); emit(); },
    removeCone(id) {
      const idx = cones.findIndex((c) => c.id === id);
      if (idx >= 0) { cones.splice(idx, 1); emit(); }
    },
    moveCone(id, x, y) {
      const c = cones.find((k) => k.id === id);
      if (c) { c.x = x; c.y = y; emit(); }
    },
    clearAll() { cones.length = 0; emit(); },
  };
}
