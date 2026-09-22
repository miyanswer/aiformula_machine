# コーン配置・検知・回避 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `web_simulator` にコーンの自由配置UI、`cone.pt`ベースの実カメラ画像コーン検知、反応的回避＋レーシングライン回避後処理、車輪スリップ誤差モデル、コーンランドマークによるオドメトリ補正を追加する。

**Architecture:** 5つの新規ファイル（`cone_props.js`, `cone_editor.js`, `cone_detector.js`, `cone_avoidance.js`, `export_cone_onnx.py`）を、既存の移植コード（`lane_navigator.js`）にはごく小さな2箇所の追加（`correctedPoseSequence`のexport、`applyExternalCorrection()`）だけを入れて組み合わせる。`simulator.js`が全部を配線する。

**Tech Stack:** three.js r160（vendored） + GLTFLoader、onnxruntime-web（vendored）、素のES modules（ビルドツールなし、`web_simulator/serve.py`で配信）、Ultralytics YOLO（Python側のONNX変換のみ）。

**Spec:** [docs/superpowers/specs/2026-09-22-cone-avoidance-design.md](../specs/2026-09-22-cone-avoidance-design.md)

## Global Constraints

- `js/lane_navigator.js`への変更は次の2つに限定する。他の関数・アルゴリズムは一切変更しない:
  1. 内部関数`correctYawDrift`を`export function correctedPoseSequence(samples, yawDrift)`（姿勢列計算部分）に分割し、`correctYawDrift`はその結果を使って再投影するだけにする。境界点(`left`/`right`)の出力値は変更前と完全に同一でなければならない。
  2. `LaneNavigator`に`applyExternalCorrection(dx, dy, dyaw, damping = 0.15)`という公開メソッドを1つ追加する。
- `optimizeRaceline`（QP本体）・`_mapMatching`・`_stepMapping`・`_stepRacing`・`BoundaryRecorder`・`buildCourseMap`のロジックは変更しない。
- コーンの当たり判定半径: `CONE_RADIUS = 0.15`（models/cone.glbの実測値）。
- 車両半幅: 既存の`js/collision.js`の`VEHICLE_HALF_WIDTH = 0.40`をimportして使う（再定義しない）。
- スリップ: 左右輪独立、時定数`SLIP_TAU_S = 2.0`秒、ノイズ`SLIP_NOISE = 0.05`、クランプ`SLIP_MAX = 0.08`（±8%）。
- 反応的回避: `REACT_MARGIN = 0.15`, `REACT_CLEARANCE = VEHICLE_HALF_WIDTH + CONE_RADIUS + REACT_MARGIN`(=0.70), `REACT_LOOKAHEAD_X = 4.0`, `REACT_MIN_X = 0.3`, `REACT_GAIN = 1.5`, `REACT_MAX_OMEGA_BIAS = 0.6`, `REACT_SLOW_X = 1.5`, `REACT_SLOW_V = 0.5`。
- レーシングライン後処理: `DEFLECT_CLEARANCE = VEHICLE_HALF_WIDTH + CONE_RADIUS + 0.20`(=0.75)。
- コーン検知の有効範囲: `0.3m < x < 8.0m`, `|y| < 3.0m`。信頼度閾値0.4、IoU 0.45。
- 実機ROSノードは作らない。`src/oit_navigation/oit_navigation/export_cone_onnx.py`はブラウザ用ONNX変換ツールであり、ROSノードではない。
- 自動テストランナー・`node`はこの環境に存在しない。すべての検証は`web_simulator/serve.py`でサーブしたページをブラウザで開き、ブラウザの開発者コンソール（このプランを実行するエージェントが使えるブラウザツール）から直接JSを実行して確認する。

---

## Task 1: コーンモデル・当たり判定 (`cone_props.js`)

**Files:**
- Create: `web_simulator/js/cone_props.js`
- Reference (read-only, パターンの参考にする): `web_simulator/js/course.js:190-200`（GLTFLoaderの使い方）, `web_simulator/js/course_props.js:1-20,185-205`（MyLapsゲートの`addMyLapsGantry`/`worldColliders`と同じ構造にする）

**Interfaces:**
- Produces:
  - `export const CONE_RADIUS = 0.15;`
  - `export const CONE_SCALE = 0.01;`
  - `export async function loadConeTemplate()` → `Promise<THREE.Object3D>`（`models/cone.glb`を1回だけロードして返す、以降は呼び出し側で`clone()`する）
  - `export function addCone(parent, template, {x, y})` → `THREE.Object3D`（`template.clone()`をposition `(x,y,0)`・スケール`CONE_SCALE`で`parent`（`rosRoot`）に追加して返す）
  - `export function coneWorldColliders(cones)` → `Array<{x:number, y:number, r:number}>`（`cones`は`{x,y}[]`、各要素に`r: CONE_RADIUS`を付けて返すだけ）

- [ ] **Step 1: `cone.glb`のロードとコライダー生成を実装する**

```js
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
```

- [ ] **Step 2: サーバーを起動し、モジュールが単独でロードできることを確認する**

```bash
python3 web_simulator/serve.py 8010
```

ブラウザで `http://localhost:8010/web_simulator/index.html` を開き、開発者コンソールで:

```js
const mod = await import('./js/cone_props.js');
console.log(mod.CONE_RADIUS, mod.CONE_SCALE);
const template = await mod.loadConeTemplate();
console.log(template.type); // "Group" or "Object3D"
```

Expected: `0.15 0.01` と `"Group"`（または`"Object3D"`）が出力され、エラーが出ないこと。

- [ ] **Step 3: `addCone`/`coneWorldColliders`の動作を確認する**

同じコンソールで:

```js
console.log(mod.coneWorldColliders([{ x: 1, y: 2 }, { x: 3, y: 4 }]));
// 期待値: [{x:1,y:2,r:0.15},{x:3,y:4,r:0.15}]
```

Expected: 上記の通りの配列が出力される。`addCone`は`simulator.js`に実際の`rosRoot`を持つTask 5で結合テストする（このタスク単体ではシーン統合なし）。

- [ ] **Step 4: Commit**

```bash
git add web_simulator/js/cone_props.js
git commit -m "$(cat <<'EOF'
feat(web_simulator): add cone model loading + collider helpers

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: `LaneNavigator`への最小限のフック追加

**Files:**
- Modify: `web_simulator/js/lane_navigator.js:370-386`（`correctYawDrift`の分割）, `web_simulator/js/lane_navigator.js:840-843`付近（`_stop()`の直後に`applyExternalCorrection`を追加）

**Interfaces:**
- Produces:
  - `export function correctedPoseSequence(samples, yawDrift)` → `Array<[number, number, number]>`（`samples`と同じ長さの補正後姿勢列）
  - `LaneNavigator.prototype.applyExternalCorrection(dx, dy, dyaw, damping = 0.15)` → `void`

- [ ] **Step 1: `correctYawDrift`を分割する（動作を変えない）**

現在のコード（`web_simulator/js/lane_navigator.js:370-386`）:

```js
function correctYawDrift(samples, yawDrift, xRec) {
  const s0 = samples[0].s, s1 = samples[samples.length - 1].s;
  const span = Math.max(s1 - s0, 1e-9);
  const poses = [[...samples[0].pose]];
  for (let i = 1; i < samples.length; i++) {
    const a = samples[i - 1], b = samples[i];
    const rot = -yawDrift * ((a.s + b.s) * 0.5 - s0) / span;
    const dx = b.pose[0] - a.pose[0], dy = b.pose[1] - a.pose[1];
    const c = Math.cos(rot), s = Math.sin(rot);
    const prev = poses[poses.length - 1];
    poses.push([prev[0] + c * dx - s * dy, prev[1] + s * dx + c * dy, b.pose[2] - yawDrift * (b.s - s0) / span]);
  }
  return {
    left: poses.map((ps, i) => vehicleToWorld(ps, xRec, samples[i].yLeft)),
    right: poses.map((ps, i) => vehicleToWorld(ps, xRec, samples[i].yRight)),
  };
}
```

これを次のように置き換える（`poses`の計算部分を`correctedPoseSequence`としてexportし、`correctYawDrift`はそれを呼ぶだけにする）:

```js
// 補正後の姿勢列だけを計算する (境界点の再投影にもコーン記憶の再投影にも
// 使う共通ロジック)。ヨードリフトを走行距離sに応じて滑らかに配分するのは
// 元のcorrectYawDriftと完全に同じ計算。
export function correctedPoseSequence(samples, yawDrift) {
  const s0 = samples[0].s, s1 = samples[samples.length - 1].s;
  const span = Math.max(s1 - s0, 1e-9);
  const poses = [[...samples[0].pose]];
  for (let i = 1; i < samples.length; i++) {
    const a = samples[i - 1], b = samples[i];
    const rot = -yawDrift * ((a.s + b.s) * 0.5 - s0) / span;
    const dx = b.pose[0] - a.pose[0], dy = b.pose[1] - a.pose[1];
    const c = Math.cos(rot), s = Math.sin(rot);
    const prev = poses[poses.length - 1];
    poses.push([prev[0] + c * dx - s * dy, prev[1] + s * dx + c * dy, b.pose[2] - yawDrift * (b.s - s0) / span]);
  }
  return poses;
}

function correctYawDrift(samples, yawDrift, xRec) {
  const poses = correctedPoseSequence(samples, yawDrift);
  return {
    left: poses.map((ps, i) => vehicleToWorld(ps, xRec, samples[i].yLeft)),
    right: poses.map((ps, i) => vehicleToWorld(ps, xRec, samples[i].yRight)),
  };
}
```

`buildCourseMap`（同ファイル388行目付近）はこの`correctYawDrift`を呼んでいる箇所を変更する必要はない（シグネチャ・戻り値とも同一）。

- [ ] **Step 2: 回帰確認 - 境界点の出力が変更前と一致することを確認する**

```bash
python3 web_simulator/serve.py 8010
```

ブラウザのコンソールで、変更前の`correctYawDrift`のロジックと同じ入力を与えて手動比較する:

```js
const mod = await import('./js/lane_navigator.js');
const samples = [
  { s: 0, pose: [0, 0, 0], yLeft: 1.5, yRight: -1.5 },
  { s: 5, pose: [5, 0.1, 0.02], yLeft: 1.4, yRight: -1.6 },
  { s: 10, pose: [10, 0.3, 0.05], yLeft: 1.5, yRight: -1.5 },
];
const poses = mod.correctedPoseSequence(samples, 0.1);
console.log(poses);
// poses[0] は samples[0].pose と同一 ([0,0,0]) であること
// poses.length === samples.length (3) であること
console.log(poses.length === samples.length, poses[0][0] === 0 && poses[0][1] === 0 && poses[0][2] === 0);
```

Expected: `true true` が出力される。

- [ ] **Step 3: `applyExternalCorrection`を追加する**

`web_simulator/js/lane_navigator.js`の`_mapMatching()`メソッドの直後（840行目付近、`_stop(dt)`の直前）に追加:

```js
  // navigator.py本体にはない外部フック: cone_avoidance.jsのコーンランドマーク
  // 照合結果をthis.corrに反映する。_mapMatching()と違い、呼び出し側
  // (coneLandmarkCorrection)が渡すdx/dyは既にmapPose座標系(this.corrを
  // 適用した後の世界座標系)での差分なので、_mapMatching()のような
  // 「生のpose座標からの逆算」は不要で、corrへの直接加算でよい。
  // _mapMatching自体は一切変更しない。
  applyExternalCorrection(dx, dy, dyaw, damping = 0.15) {
    this.corr = [
      this.corr[0] + dx * damping,
      this.corr[1] + dy * damping,
      this.corr[2] + dyaw * damping,
    ];
  }
```

- [ ] **Step 4: `applyExternalCorrection`の動作確認**

```js
const { LaneNavigator } = await import('./js/lane_navigator.js');
const nav = new LaneNavigator();
console.log(nav.corr); // [0, 0, 0]
nav.applyExternalCorrection(1.0, 0.0, 0.0, 0.5);
console.log(nav.corr); // [0.5, 0, 0] (dx=1.0 x damping=0.5)
```

Expected: 2回目の`console.log`で`[0.5, 0, 0]`（またはこれに極めて近い浮動小数点値）が出力される。

- [ ] **Step 5: Commit**

```bash
git add web_simulator/js/lane_navigator.js
git commit -m "$(cat <<'EOF'
feat(web_simulator): expose correctedPoseSequence + applyExternalCorrection

Minimal, behavior-preserving hooks on the ported LaneNavigator so
cone_avoidance.js can reuse the exact same drift-correction math
without duplicating or modifying the ported algorithm itself.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: 車輪スリップモデル (`vehicle_physics.js`)

**Files:**
- Modify: `web_simulator/js/vehicle_physics.js`（全体、特に45-100行目のクラス定義、142-149行目の`wheelSpeeds()`）

**Interfaces:**
- Consumes: なし（既存の`VehiclePhysics`クラスのみ）
- Produces:
  - `VehiclePhysics.prototype.measuredWheelSpeeds()` → `{left:number, right:number}`（スリップ込み）
  - `VehiclePhysics.prototype.slipL`, `.slipR`（現在のスリップ率、-0.08〜0.08）
  - 既存の`wheelSpeeds()`は変更しない（真値のまま）

- [ ] **Step 1: スリップの状態変数と更新関数を追加する**

`web_simulator/js/vehicle_physics.js`の`export class VehiclePhysics {`直後（現55行目の`constructor`内）と、`step()`/`stepAutonomous()`の末尾に追記する。ファイル冒頭の定数群（26行目付近、`ANGULAR_DAMPING`の後）に追加:

```js
// 車輪スリップ誤差モデル (CLAUDE.md: 「機体はスリップ誤差が8%程度ある」)。
// 左右輪独立に、時定数付きランダムウォークで±8%以内のスリップ率を持たせる。
// 真の物理位置(this.x/y/yaw)には影響させず、measuredWheelSpeeds()だけに
// 反映することで、CAN配信・オドメトリ推定(js/simulator.jsのlocalizer)と
// 真の位置が実車と同じように乖離していくようにする。
const SLIP_TAU_S = 2.0; // [s] 時定数
const SLIP_NOISE = 0.05; // [1/sqrt(s)] ノイズ強度
const SLIP_MAX = 0.08; // ±8%にクランプ
```

`constructor()`（55行目付近）に追加:

```js
    this.slipL = 0; // 左輪スリップ率 (-0.08〜0.08、measuredWheelSpeeds()だけに影響)
    this.slipR = 0; // 右輪スリップ率
```

`reset()`（55-62行目）にも同様に追加:

```js
    this.slipL = 0;
    this.slipR = 0;
```

新規メソッドを`wheelSpeeds()`（142-149行目）の直前に追加:

```js
  // Box-Mullerで標準正規乱数を1つ作る (スリップのランダムウォークのノイズ項)。
  static _randn() {
    const u1 = Math.max(Math.random(), 1e-9);
    const u2 = Math.random();
    return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
  }

  // 左右輪のスリップ率を1ステップ進める。step()/stepAutonomous()の末尾から呼ぶ。
  _stepSlip(dt) {
    this.slipL += (-this.slipL / SLIP_TAU_S + SLIP_NOISE * VehiclePhysics._randn()) * dt;
    this.slipR += (-this.slipR / SLIP_TAU_S + SLIP_NOISE * VehiclePhysics._randn()) * dt;
    this.slipL = clamp(this.slipL, -SLIP_MAX, SLIP_MAX);
    this.slipR = clamp(this.slipR, -SLIP_MAX, SLIP_MAX);
  }
```

`wheelSpeeds()`の直後（150行目付近、`targetWheelSpeeds`の前）に追加:

```js
  // CAN RPM配信・オドメトリ推定(js/simulator.jsのlocalizer)が使う、
  // スリップ込みの「計測される」車輪速度。wheelSpeeds()(真値、当たり判定
  // や描画に使う)とは別に用意し、両者の乖離が8%程度のスリップを再現する。
  measuredWheelSpeeds() {
    const { left, right } = this.wheelSpeeds();
    return { left: left * (1 + this.slipL), right: right * (1 + this.slipR) };
  }
```

`step(keys, dt)`の末尾（96-100行目、yaw正規化の直後）に追加:

```js
    this._stepSlip(dt);
```

`stepAutonomous(vCmd, omegaCmd, dt)`の末尾（136-140行目、yaw正規化の直後）にも同様に追加:

```js
    this._stepSlip(dt);
```

- [ ] **Step 2: スリップが±8%に収まり、`wheelSpeeds()`(真値)には影響しないことを確認する**

```bash
python3 web_simulator/serve.py 8010
```

ブラウザのコンソールで:

```js
const { VehiclePhysics } = await import('./js/vehicle_physics.js');
const p = new VehiclePhysics();
for (let i = 0; i < 2000; i++) p.step({ forward: true, backward: false, left: false, right: false }, 0.02);
console.log('slipL', p.slipL, 'slipR', p.slipR);
console.log('within bounds', Math.abs(p.slipL) <= 0.08 && Math.abs(p.slipR) <= 0.08);
const truth = p.wheelSpeeds();
const measured = p.measuredWheelSpeeds();
console.log('true', truth, 'measured', measured);
console.log('differs', truth.left !== measured.left || truth.right !== measured.right);
```

Expected: `within bounds` が `true`。`differs` が `true`（スリップが完全に0でない限りほぼ確実に成立する。2000ステップ=40秒走らせているので、スリップが0のままで終わる確率は無視できる）。

- [ ] **Step 3: Commit**

```bash
git add web_simulator/js/vehicle_physics.js
git commit -m "$(cat <<'EOF'
feat(web_simulator): model ~8% wheel slip in vehicle physics

Adds a per-wheel, time-constant random walk slip ratio consumed only
by measuredWheelSpeeds() (CAN/odometry path); wheelSpeeds() (ground
truth, used by rendering/collision) is unchanged.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: スリップをCAN配信・オドメトリ推定に配線する (`simulator.js`)

**Files:**
- Modify: `web_simulator/js/simulator.js:1533-1539`（`publishVehicleInfoCan`）, `web_simulator/js/simulator.js:1725-1727`（`integrateLocalizer`呼び出し）

**Interfaces:**
- Consumes: `physics.measuredWheelSpeeds()`（Task 3）, `VEHICLE.track`（既存の`vehicle_physics.js`のエクスポート）

- [ ] **Step 1: `publishVehicleInfoCan`を`measuredWheelSpeeds()`に切り替える**

現在のコード（`web_simulator/js/simulator.js:1533-1539`）:

```js
function publishVehicleInfoCan() {
  if (!canTopic) return;

  const { left, right } = physics.wheelSpeeds();
  const wheelCircumference = CAN_WHEEL_DIAMETER * Math.PI;
  const toRpm = (speedMetersPerSecond) => (speedMetersPerSecond / wheelCircumference) * 60;
  const data = [...int32ToLittleEndianBytes(toRpm(right)), ...int32ToLittleEndianBytes(toRpm(left))];
```

`physics.wheelSpeeds()` を `physics.measuredWheelSpeeds()` に変更する（この関数の残りの行は変更しない）。

- [ ] **Step 2: `integrateLocalizer`をスリップ込みの計測値から呼ぶようにする**

現在のコード（`web_simulator/js/simulator.js:1725-1727`）:

```js
    // odom_imu_localizer stand-in: wheel speed + IMU yaw rate dead reckoning.
    integrateLocalizer(physics.v, physics.omega, dt);
    recordLocalizerTrail();
```

これを次のように変更する:

```js
    // odom_imu_localizer stand-in: wheel speed + IMU yaw rate dead reckoning.
    // CAN(RPM)相当のmeasuredWheelSpeeds()から v/omega を再構成する -- 真の
    // physics.v/omegaではなく、8%スリップが乗った計測値を使うことで、
    // 実車と同じようにオドメトリ推定(localizer)が真の位置からズレていく。
    const measured = physics.measuredWheelSpeeds();
    const halfTrack = VEHICLE.track / 2;
    const vMeas = (measured.left + measured.right) / 2;
    const omegaMeas = (measured.right - measured.left) / (2 * halfTrack);
    integrateLocalizer(vMeas, omegaMeas, dt);
    recordLocalizerTrail();
```

`VEHICLE`は既に`js/simulator.js`が`vehicle_physics.js`からimportしている（既存の`import { VEHICLE, ... } from './vehicle_physics.js';`相当の行を確認し、なければ`VEHICLE`をimportに追加する）。

- [ ] **Step 3: 真の位置とオドメトリ推定位置が走行とともに乖離することを確認する**

```bash
python3 web_simulator/serve.py 8010
```

ブラウザで `index.html` を開き、コンソールで（`window.__sim`は既存のデバッグフック、`physics`/`localizer`を公開している）:

```js
window.__sim.physics.reset ? window.__sim.physics.reset() : null;
for (let i = 0; i < 500; i++) {
  window.__sim.physics.step({ forward: true, backward: false, left: false, right: true }, 0.02);
}
console.log('true', window.__sim.physics.x, window.__sim.physics.y);
```

`integrateLocalizer`はモジュールスコープのprivate関数なので直接は呼べないが、実走行（W+Dキーで少なくとも15秒程度旋回走行）で確認する: ブラウザのHUD「詳細」タブの「Odometry」の位置と、HUDの「位置」（真の位置）表示を見比べ、数十秒走行後に両者が完全には一致しない（小さくズレている）ことを確認する。

Expected: 15秒以上走行後、HUDの「位置」（真値）と「詳細」タブのOdometry位置表示が完全には一致しない（cm〜数十cmオーダーでズレている）。

- [ ] **Step 4: Commit**

```bash
git add web_simulator/js/simulator.js
git commit -m "$(cat <<'EOF'
feat(web_simulator): route wheel slip into CAN publish + odometry

publishVehicleInfoCan() and the odom_imu_localizer stand-in now read
measuredWheelSpeeds() instead of the ground-truth wheelSpeeds(), so
the estimated pose drifts from the true pose like the real vehicle.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: コーン配置エディタ (`cone_editor.js`) + HUD配線

**Files:**
- Create: `web_simulator/js/cone_editor.js`
- Modify: `web_simulator/index.html:296-314`（走行タブに配置トグル・全消去ボタンを追加）
- Modify: `web_simulator/js/simulator.js`（`cone_editor.js`/`cone_props.js`の結合、クリック/ドラッグのイベント配線）

**Interfaces:**
- Consumes: `loadConeTemplate`, `addCone`, `CONE_RADIUS`（Task 1の`cone_props.js`）
- Produces:
  - `export function createConeEditor({ raycastTarget, rosRoot, camera, domElement, onChange })` → `{ enable(), disable(), isEnabled(): boolean, cones: Array<{id:string,x:number,y:number}>, addCone(x,y), removeCone(id), moveCone(id,x,y), clearAll() }`
  - `localStorage`キー: `aiformula_cones_v1`

- [ ] **Step 1: `cone_editor.js`を実装する**

```js
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
    return arr.filter((c) => c && typeof c.id === 'string' && Number.isFinite(c.x) && Number.isFinite(c.y));
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
 *   domElement: HTMLElement, onChange: (cones: Array<{id:string,x:number,y:number}>) => void}} opts
 */
export function createConeEditor({ raycastTarget, rosRoot, camera, domElement, onChange }) {
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
    const gp = groundPointFromEvent(evt);
    if (!gp) return;
    downPos = { clientX: evt.clientX, clientY: evt.clientY };
    moved = false;
    const hit = findNear(gp.x, gp.y);
    dragId = hit ? hit.id : null;
    if (dragId) evt.stopPropagation();
  }

  function onPointerMove(evt) {
    if (!enabled || dragId === null || !downPos) return;
    const dx = evt.clientX - downPos.clientX, dy = evt.clientY - downPos.clientY;
    if (Math.hypot(dx, dy) < DRAG_THRESHOLD_PX) return;
    moved = true;
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
    dragId = null; downPos = null; moved = false;
    if (wasDragId && !wasMoved) {
      // クリック(ドラッグなし) = 削除
      const idx = cones.findIndex((c) => c.id === wasDragId);
      if (idx >= 0) { cones.splice(idx, 1); emit(); }
      evt.stopPropagation();
      return;
    }
    if (wasDragId) return; // ドラッグ終了、emitは既にonPointerMoveで済み
    const gp = groundPointFromEvent(evt);
    if (!gp) return;
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
```

- [ ] **Step 2: HUDに配置トグル・全消去ボタンを追加する**

`web_simulator/index.html`の走行タブ（296-314行目、`#view-controls`の直後）に追加:

```html
      <div id="cone-editor-controls" class="btn-row">
        <button id="cone-place-btn" title="ON中はコース上のクリックでコーン追加、既存コーンのクリックで削除、ドラッグで移動">コーン配置: OFF</button>
        <button id="cone-clear-btn" class="quiet" title="配置したコーンを全消去">コーン全消去</button>
      </div>
```

- [ ] **Step 3: `simulator.js`にコーンエディタを結合する**

`web_simulator/js/simulator.js`の冒頭のimport群に追加:

```js
import { createConeEditor } from './cone_editor.js';
import { loadConeTemplate, addCone, coneWorldColliders } from './cone_props.js';
```

`rosRoot`・`camera`・`renderer.domElement`・`ground`（asphaltメッシュ、`course.js`の戻り値または既存のground変数）が定義された後（`course`ロード完了直後、既存の`addMyLapsGantry(...)`呼び出しの近く）に追加:

```js
const coneTemplate = await loadConeTemplate();
let coneInstances = []; // three.jsオブジェクト、コーン変更のたびに作り直す
let coneColliders = []; // collision.jsに渡す円コライダー

function rebuildConeInstances(cones) {
  for (const inst of coneInstances) rosRoot.remove(inst);
  coneInstances = cones.map((c) => addCone(rosRoot, coneTemplate, c));
  coneColliders = coneWorldColliders(cones);
}

const coneEditor = createConeEditor({
  raycastTarget: ground, // simulator.js既存の200x200背景プレーン(104行目付近)、sceneの直接の子
  rosRoot, // ground自体はrosRootの子ではないので、ヒット点をROS座標に戻すために渡す
  camera,
  domElement: renderer.domElement,
  onChange: rebuildConeInstances,
});
rebuildConeInstances(coneEditor.cones); // ページ読み込み時、保存済みコーンを復元
```

`applyCollisionAndDeparture()`（既存の衝突判定関数）が組み立てる`obstacles`配列に`coneColliders`を追加する（既存のMyLapsコライダー配列と同様の書き方に合わせて、その配列リテラルに`...coneColliders`を足す）。

HUDボタンの配線（既存の`autonomous-btn`等のイベントリスナーが並んでいる箇所の近くに追加）:

```js
const conePlaceBtn = document.getElementById('cone-place-btn');
const coneClearBtn = document.getElementById('cone-clear-btn');
conePlaceBtn.addEventListener('click', () => {
  if (coneEditor.isEnabled()) { coneEditor.disable(); conePlaceBtn.textContent = 'コーン配置: OFF'; }
  else { coneEditor.enable(); conePlaceBtn.textContent = 'コーン配置: ON'; }
});
coneClearBtn.addEventListener('click', () => coneEditor.clearAll());
```

`window.__sim`のデバッグオブジェクトに`coneEditor`を追加する（既存の`Object.assign(window.__sim, {...})`または同等の初期化箇所に`coneEditor`を足す）。

- [ ] **Step 4: ブラウザでクリック配置・ドラッグ移動・クリック削除・永続化を確認する**

```bash
python3 web_simulator/serve.py 8010
```

ブラウザで`index.html`を開き、ブラウザ操作ツールで:
1. 「走行」タブの「コーン配置: OFF」ボタンをクリックし「ON」になることを確認。
2. コース上の空いた場所をクリック。`window.__sim.coneEditor.cones.length`が1増えることをコンソールで確認。
3. 追加したコーンをドラッグして別の場所に移動。`x`/`y`が変わることを確認。
4. 同じコーンをドラッグなしでクリック。`cones.length`が1減ることを確認。
5. 「コーン全消去」をクリックし`cones.length === 0`になることを確認。
6. コーンを1つ追加した状態でページをリロードし、`window.__sim.coneEditor.cones.length === 1`のままであることを確認（localStorage永続化）。

Expected: 上記1〜6すべてが期待通りに動作する。

- [ ] **Step 5: Commit**

```bash
git add web_simulator/js/cone_editor.js web_simulator/index.html web_simulator/js/simulator.js
git commit -m "$(cat <<'EOF'
feat(web_simulator): add click/drag cone placement editor

Click to add, drag to move, click-without-drag to remove, unlimited
count, persisted to localStorage. Wired into collision.js's obstacle
list via cone_props.js's colliders.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: コーンONNX変換ツール (`export_cone_onnx.py`)

**Files:**
- Create: `src/oit_navigation/oit_navigation/export_cone_onnx.py`
- Modify: `web_simulator/README.md`（生成手順の追記、UFLDの節と同じ場所）

**Interfaces:**
- 入力: `models/cone.pt`（Ultralytics YOLO checkpoint）
- 出力: `web_simulator/models/cone.onnx`

- [ ] **Step 1: 変換スクリプトを実装する**

```python
#!/usr/bin/env python3
"""
export_cone_onnx.py - cone.pt (Ultralytics YOLO 検出モデル) を
web_simulator 用のONNXへ変換する。

ROSノードではない。export_onnx_web.py (YOLOP) / export_ufld_onnx と同じ、
ブラウザ実行用アセットを書き出すだけのツール。web_simulator/js/cone_detector.js
がonnxruntime-webでこの出力を読み込む。

cone.pt はYOLOPと違い素のUltralytics YOLO検出モデルなので、YOLOPのような
カスタムラッパーは不要 -- Ultralytics自身のexport(format="onnx")を使う。

Usage:
    python3 export_cone_onnx.py \
        --weights /aiformula_machine/models/cone.pt \
        --output /aiformula_machine/web_simulator/models/cone.onnx
"""
import argparse
import shutil

from ultralytics import YOLO

DEFAULT_INPUT_SIZE = 640


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", default="/aiformula_machine/models/cone.pt")
    parser.add_argument("--output", default="/aiformula_machine/web_simulator/models/cone.onnx")
    parser.add_argument("--size", type=int, default=DEFAULT_INPUT_SIZE, help="Square input side length")
    parsed = parser.parse_args(args=args)

    model = YOLO(parsed.weights)
    exported_path = model.export(format="onnx", imgsz=parsed.size, opset=12, simplify=True)
    shutil.move(str(exported_path), parsed.output)
    print(f"[export_cone_onnx] Wrote ONNX graph: {parsed.output} (input: 1x3x{parsed.size}x{parsed.size})")


if __name__ == "__main__":
    main()
```

`setup.py`のentry_points（`export_onnx_web`や`export_ufld_onnx`が登録されている箇所）に、同じパターンで`export_cone_onnx`を追加する。`src/oit_navigation/setup.py`を開き、既存の`'export_onnx_web = oit_navigation.export_onnx_web:main'`のような行の隣に`'export_cone_onnx = oit_navigation.export_cone_onnx:main'`を追加する。

- [ ] **Step 2: 構文チェック（このサンドボックスにROS/ultralytics環境がない前提の最小確認）**

```bash
python3 -m py_compile src/oit_navigation/oit_navigation/export_cone_onnx.py
```

Expected: エラーなく終了する（`ultralytics`が未インストールの環境でも`py_compile`は構文チェックのみなので通る）。

実際に`cone.onnx`を生成するには、`ultralytics`・`torch`が入ったROS環境（ユーザーの開発機）で以下を実行する必要がある。このタスクではスクリプトの作成までとし、実際の生成と動作確認はTask 7で行う（`cone.onnx`が存在しない場合のエラーハンドリングをTask 7で作るため、Task 7はこのファイルが未生成でも進められる）:

```bash
ros2 run oit_navigation export_cone_onnx
```

- [ ] **Step 3: READMEに生成手順を追記する**

`web_simulator/README.md`の、UFLDの生成コマンドが書かれている節（既存の「`models/ufld_honda_finetuned_best.pth` を置いて `ros2 run oit_navigation export_ufld_onnx` で生成する」という記述の近く）に、同じ形式で追記する:

```markdown
  - **コーン検知**: `models/cone.onnx`（[`js/cone_detector.js`](js/cone_detector.js)）。git管理外のため、
    `models/cone.pt` を置いて `ros2 run oit_navigation export_cone_onnx` で生成する。
```

- [ ] **Step 4: Commit**

```bash
git add src/oit_navigation/oit_navigation/export_cone_onnx.py src/oit_navigation/setup.py web_simulator/README.md
git commit -m "$(cat <<'EOF'
feat(oit_navigation): add cone.pt -> ONNX export tool for the browser sim

Not a ROS node -- same category as export_onnx_web.py/export_ufld_onnx,
a build-time asset conversion script for web_simulator/js/cone_detector.js.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: ブラウザ側コーン検知 (`cone_detector.js`)

**Files:**
- Create: `web_simulator/js/cone_detector.js`
- Reference (読み取りのみ、実装パターンの参考): `web_simulator/js/lane_model_detector.js`（WebGPU→WASMフォールバックの書き方）

**Interfaces:**
- Consumes: `projectToGround`, `DEFAULT_CAMERA`（`lane_navigator.js`、既存export）
- Produces:
  - `export class ConeDetector { async load(onnxUrl, wasmDir); async infer(sourceCanvas): Promise<Array<{x:number, y:number, conf:number}>>; get session(); }`
  - `infer()`の戻り値は既にground座標系（車体フレームx=前方, y=左）に変換済みで、5章の有効範囲（`0.3<x<8.0`, `|y|<3.0`）でフィルタ済みのものを返す。

- [ ] **Step 1: 純粋なデコード/NMSロジックを実装する（`session`なしでテストできる部分）**

```js
// web_simulator/js/cone_detector.js
// cone.onnx (Ultralytics YOLO検出, export_cone_onnx.pyで生成) を
// onnxruntime-webでブラウザ推論する。js/lane_model_detector.jsと同じ
// WebGPU->WASMフォールバック構成。コーンは接地物なので、信号機の
// (traffic_light_distance_node.pyのような)占有率からの距離逆算は不要 --
// バウンディングボックス下辺中央を lane_navigator.js の projectToGround()
// (白線検知と同じカメラモデル)にそのまま渡せば車体フレームの地面座標が
// 直接求まる。

import { projectToGround, DEFAULT_CAMERA } from './lane_navigator.js';

const MODEL_INPUT_SIZE = 640;
const CONF_THRESHOLD = 0.4;
const IOU_THRESHOLD = 0.45;
const VALID_X_MIN = 0.3, VALID_X_MAX = 8.0, VALID_Y_ABS_MAX = 3.0;

function iou(a, b) {
  const x1 = Math.max(a.x1, b.x1), y1 = Math.max(a.y1, b.y1);
  const x2 = Math.min(a.x2, b.x2), y2 = Math.min(a.y2, b.y2);
  const inter = Math.max(0, x2 - x1) * Math.max(0, y2 - y1);
  const areaA = (a.x2 - a.x1) * (a.y2 - a.y1);
  const areaB = (b.x2 - b.x1) * (b.y2 - b.y1);
  return inter / Math.max(areaA + areaB - inter, 1e-9);
}

/**
 * 単純なgreedy NMS。
 * @param {Array<{x1,y1,x2,y2,conf,cls}>} boxes
 * @returns {Array<{x1,y1,x2,y2,conf,cls}>}
 */
export function nonMaxSuppression(boxes, iouThreshold = IOU_THRESHOLD) {
  const sorted = [...boxes].sort((a, b) => b.conf - a.conf);
  const kept = [];
  for (const b of sorted) {
    if (kept.every((k) => iou(k, b) < iouThreshold)) kept.push(b);
  }
  return kept;
}

/**
 * Ultralytics YOLO ONNXの標準出力 [1, 4+numClasses, numBoxes] をデコードする。
 * @param {Float32Array} data
 * @param {number} numBoxes (例: 8400)
 * @param {number} numClasses
 * @param {number} confThreshold
 * @returns {Array<{x1,y1,x2,y2,conf,cls}>} 640x640モデル座標系のボックス
 */
export function decodeYoloOutput(data, numBoxes, numClasses, confThreshold = CONF_THRESHOLD) {
  const boxes = [];
  for (let i = 0; i < numBoxes; i++) {
    let bestCls = 0, bestConf = 0;
    for (let c = 0; c < numClasses; c++) {
      const conf = data[(4 + c) * numBoxes + i];
      if (conf > bestConf) { bestConf = conf; bestCls = c; }
    }
    if (bestConf < confThreshold) continue;
    const cx = data[0 * numBoxes + i], cy = data[1 * numBoxes + i];
    const w = data[2 * numBoxes + i], h = data[3 * numBoxes + i];
    boxes.push({ x1: cx - w / 2, y1: cy - h / 2, x2: cx + w / 2, y2: cy + h / 2, conf: bestConf, cls: bestCls });
  }
  return nonMaxSuppression(boxes);
}

/**
 * モデル座標(640x640, レターボックス)のボックスを、元画像ピクセル座標に戻す。
 */
export function unletterbox(box, srcWidth, srcHeight, modelSize = MODEL_INPUT_SIZE) {
  const scale = Math.min(modelSize / srcWidth, modelSize / srcHeight);
  const padX = (modelSize - srcWidth * scale) / 2;
  const padY = (modelSize - srcHeight * scale) / 2;
  return {
    x1: (box.x1 - padX) / scale, y1: (box.y1 - padY) / scale,
    x2: (box.x2 - padX) / scale, y2: (box.y2 - padY) / scale,
    conf: box.conf, cls: box.cls,
  };
}

/**
 * バウンディングボックス下辺中央を地面に投影し、有効範囲でフィルタする。
 * @param {Array<{x1,y1,x2,y2,conf}>} boxes 元画像ピクセル座標
 * @param {number} width 元画像幅
 * @param {number} height 元画像高さ
 * @returns {Array<{x:number, y:number, conf:number}>}
 */
export function groundPositionsFromBoxes(boxes, width, height) {
  const out = [];
  for (const b of boxes) {
    const u = [(b.x1 + b.x2) / 2];
    const v = [b.y2];
    const g = projectToGround(DEFAULT_CAMERA, u, v, width, height);
    if (g.x.length === 0) continue;
    const x = g.x[0], y = g.y[0];
    if (x < VALID_X_MIN || x > VALID_X_MAX || Math.abs(y) > VALID_Y_ABS_MAX) continue;
    out.push({ x, y, conf: b.conf });
  }
  return out;
}
```

- [ ] **Step 2: ONNX推論クラスを追加する（`lane_model_detector.js`と同じフォールバック構成）**

同ファイルの末尾に追加:

```js
export class ConeDetector {
  constructor() {
    this.session = null;
    this._letterboxCanvas = document.createElement('canvas');
    this._letterboxCanvas.width = MODEL_INPUT_SIZE;
    this._letterboxCanvas.height = MODEL_INPUT_SIZE;
    this._letterboxCtx = this._letterboxCanvas.getContext('2d', { willReadFrequently: true });
  }

  async load(onnxUrl, wasmDir) {
    // eslint-disable-next-line no-undef -- js/lane_model_detector.jsと同じ、
    // index.htmlの<script>タグで読み込まれるortグローバル。
    ort.env.wasm.wasmPaths = new URL(wasmDir, document.baseURI).href;
    try {
      this.session = await ort.InferenceSession.create(onnxUrl, {
        executionProviders: ['webgpu'],
        graphOptimizationLevel: 'all',
      });
    } catch (err) {
      console.warn('ConeDetector: WebGPU EP unavailable, falling back to CPU WASM', err);
      this.session = await ort.InferenceSession.create(onnxUrl, {
        executionProviders: ['wasm'],
        graphOptimizationLevel: 'all',
      });
    }
  }

  /**
   * @param {HTMLCanvasElement|OffscreenCanvas} sourceCanvas
   * @returns {Promise<Array<{x:number, y:number, conf:number}>>} 車体フレーム地面座標、有効範囲フィルタ済み
   */
  async infer(sourceCanvas) {
    if (!this.session) throw new Error('ConeDetector.load() must complete before infer()');
    const width = sourceCanvas.width, height = sourceCanvas.height;
    const scale = Math.min(MODEL_INPUT_SIZE / width, MODEL_INPUT_SIZE / height);
    const drawW = width * scale, drawH = height * scale;
    const padX = (MODEL_INPUT_SIZE - drawW) / 2, padY = (MODEL_INPUT_SIZE - drawH) / 2;

    this._letterboxCtx.fillStyle = '#727272';
    this._letterboxCtx.fillRect(0, 0, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE);
    this._letterboxCtx.drawImage(sourceCanvas, 0, 0, width, height, padX, padY, drawW, drawH);
    const image = this._letterboxCtx.getImageData(0, 0, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE);

    const planeSize = MODEL_INPUT_SIZE * MODEL_INPUT_SIZE;
    const input = new Float32Array(3 * planeSize);
    for (let i = 0; i < planeSize; i++) {
      const idx = i * 4;
      input[i] = image.data[idx] / 255;
      input[planeSize + i] = image.data[idx + 1] / 255;
      input[2 * planeSize + i] = image.data[idx + 2] / 255;
    }
    const inputTensor = new ort.Tensor('float32', input, [1, 3, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE]);
    const results = await this.session.run({ images: inputTensor });
    const outputKey = Object.keys(results)[0];
    const output = results[outputKey];
    const numClasses = output.dims[1] - 4;
    const numBoxes = output.dims[2];

    const modelBoxes = decodeYoloOutput(output.data, numBoxes, numClasses);
    const pixelBoxes = modelBoxes.map((b) => unletterbox(b, width, height));
    return groundPositionsFromBoxes(pixelBoxes, width, height);
  }
}
```

- [ ] **Step 3: 純粋関数(デコード/NMS/座標変換)を合成データで確認する**

```bash
python3 web_simulator/serve.py 8010
```

ブラウザのコンソールで:

```js
const mod = await import('./js/cone_detector.js');

// NMS: 重なった2つの箱は1つに絞られる
const boxes = [
  { x1: 0, y1: 0, x2: 10, y2: 10, conf: 0.9, cls: 0 },
  { x1: 1, y1: 1, x2: 11, y2: 11, conf: 0.5, cls: 0 },
  { x1: 100, y1: 100, x2: 110, y2: 110, conf: 0.8, cls: 0 },
];
console.log(mod.nonMaxSuppression(boxes).length); // 期待値: 2

// unletterbox: パディングなし(width===height===640)なら値がそのまま返る
console.log(mod.unletterbox({ x1: 100, y1: 100, x2: 200, y2: 200, conf: 1, cls: 0 }, 640, 640));
```

Expected: `2` が出力される。`unletterbox`の出力は`{x1:100,y1:100,x2:200,y2:200,...}`と一致する（scale=1, pad=0のため）。

`session`を使う`load()`/`infer()`は`models/cone.onnx`が存在する環境でのみ実行できるため、このタスクでは自動確認しない（Task 9でエラーハンドリングとあわせて確認する）。

- [ ] **Step 4: Commit**

```bash
git add web_simulator/js/cone_detector.js
git commit -m "$(cat <<'EOF'
feat(web_simulator): add browser-side cone detector (cone.onnx)

Mirrors lane_model_detector.js's onnxruntime-web WebGPU/WASM fallback.
Ground position comes from projectToGround() on the bbox bottom-center
pixel (cones are grounded, unlike the traffic light's elevated target).

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: 回避ロジックの純粋関数群 (`cone_avoidance.js`)

**Files:**
- Create: `web_simulator/js/cone_avoidance.js`

**Interfaces:**
- Consumes: `VEHICLE_HALF_WIDTH`（`collision.js`）, `CONE_RADIUS`（`cone_props.js`）, `vehicleToWorld`, `rateLimit`, `correctedPoseSequence`, `RacelineFollower`（`lane_navigator.js`）, `applyExternalCorrection`/`mapPose`（`LaneNavigator`インスタンスメソッド、Task 2）
- Produces:
  - `export function reactiveAvoid(cmd, detections, prevBias, dt, maxRate = 4.0)` → `{v, omega, bias}`
  - `export class ConeRecorder { update(s, pose, detections); finalize(samples, yawDrift): Array<{x,y}>; }`
  - `export function deflectRacelineAroundCones(points, speeds, cones)` → `{points, speeds}`
  - `export function coneLandmarkCorrection(navigator, localizerPose, detections, coneMapPoints, gate = 1.0)` → `void`（`navigator.applyExternalCorrection`を内部で呼ぶ）

- [ ] **Step 1: 定数と`reactiveAvoid`を実装する**

```js
// web_simulator/js/cone_avoidance.js
// コーン回避: 反応的ナッジ(毎フレーム)、1周目のコーン記憶、2周目レーシング
// ラインへの回避後処理、コーンランドマークによるオドメトリ補正。
// js/lane_navigator.js (実機のnavigator.py/course_map.py/raceline_qp.pyの
// 移植)本体は一切変更しない -- ここにある関数はすべて外側から結果を使う/
// 上書きするだけ。詳細はdocs/superpowers/specs/2026-09-22-cone-avoidance-design.md参照。

import { VEHICLE_HALF_WIDTH } from './collision.js';
import { CONE_RADIUS } from './cone_props.js';
import { vehicleToWorld, rateLimit, correctedPoseSequence, RacelineFollower } from './lane_navigator.js';

// --- 7.1: 反応的回避 ---
const REACT_MARGIN = 0.15;
export const REACT_CLEARANCE = VEHICLE_HALF_WIDTH + CONE_RADIUS + REACT_MARGIN; // 0.70
const REACT_LOOKAHEAD_X = 4.0;
const REACT_MIN_X = 0.3;
const REACT_GAIN = 1.5;
const REACT_MAX_OMEGA_BIAS = 0.6;
const REACT_SLOW_X = 1.5;
const REACT_SLOW_V = 0.5;

/**
 * navigator.step()が返したcmdに、検出中のコーンを避ける操舵バイアスを
 * 加えて返す。MAPPING/RACING両方で毎フレーム呼ぶ。
 * @param {{v:number, omega:number}} cmd
 * @param {Array<{x:number, y:number, conf:number}>} detections 車体フレーム
 * @param {number} prevBias 前回返したbias (レート制限のため)
 * @param {number} dt
 * @param {number} maxRate [rad/s^2] biasの変化率上限
 */
export function reactiveAvoid(cmd, detections, prevBias, dt, maxRate = 4.0) {
  let worstPush = 0, pushSign = 0, worstX = REACT_MIN_X, closeSlow = false;
  for (const d of detections) {
    if (d.x < REACT_MIN_X || d.x > REACT_LOOKAHEAD_X) continue;
    const shortfall = REACT_CLEARANCE - Math.abs(d.y);
    if (shortfall > worstPush) { worstPush = shortfall; pushSign = d.y >= 0 ? -1 : 1; worstX = d.x; }
    if (d.x < REACT_SLOW_X && Math.abs(d.y) < REACT_CLEARANCE) closeSlow = true;
  }
  let targetBias = 0;
  if (worstPush > 0) {
    targetBias = Math.max(-REACT_MAX_OMEGA_BIAS, Math.min(REACT_MAX_OMEGA_BIAS,
      pushSign * REACT_GAIN * worstPush / Math.max(worstX, 0.5)));
  }
  const bias = rateLimit(prevBias, targetBias, maxRate, dt);
  return {
    v: closeSlow ? Math.min(cmd.v, REACT_SLOW_V) : cmd.v,
    omega: cmd.omega + bias,
    bias,
  };
}
```

- [ ] **Step 2: `reactiveAvoid`の挙動を合成データで確認する**

```bash
python3 web_simulator/serve.py 8010
```

```js
const mod = await import('./js/cone_avoidance.js');

// 真正面(x=1.5, y=0)にコーン -> 左右どちらかに大きくバイアスがかかる
const r1 = mod.reactiveAvoid({ v: 1.0, omega: 0 }, [{ x: 1.5, y: 0, conf: 0.9 }], 0, 0.02);
console.log('front cone', r1, Math.abs(r1.omega) > 0, r1.v < 1.0);

// 検出なし -> biasは0に向かう、速度はそのまま
const r2 = mod.reactiveAvoid({ v: 1.0, omega: 0.1 }, [], r1.bias, 0.02);
console.log('no detections', r2, r2.v === 1.0);

// 遠く(x=6)にコーン -> 無視される(REACT_LOOKAHEAD_X=4.0の範囲外)
const r3 = mod.reactiveAvoid({ v: 1.0, omega: 0 }, [{ x: 6, y: 0, conf: 0.9 }], 0, 0.02);
console.log('far cone ignored', r3.omega === 0, r3.v === 1.0);
```

Expected: `front cone` の行で`true true`（omegaが0でない、速度が減速）。`no detections`の行で`true`。`far cone ignored`の行で`true true`。

- [ ] **Step 3: `ConeRecorder`を実装する**

同ファイルに追加:

```js
// --- 7.2: 1周目のコーン記憶 ---
export class ConeRecorder {
  constructor(gateRadius = 0.6) { this.cones = []; this.gateRadius = gateRadius; }

  /**
   * @param {number} s localizer.s (走行距離)
   * @param {[number, number, number]} pose [localizer.x, localizer.y, localizer.yaw]
   * @param {Array<{x:number, y:number}>} detections 車体フレーム
   */
  update(s, pose, detections) {
    for (const d of detections) {
      const [wx, wy] = vehicleToWorld(pose, d.x, d.y); // 名寄せ用の未補正ラフ座標
      const hit = this.cones.find((c) => Math.hypot(c.roughX - wx, c.roughY - wy) < this.gateRadius);
      if (!hit) {
        this.cones.push({ roughX: wx, roughY: wy, s, localX: d.x, localY: d.y });
      } else if (d.x < hit.localX) {
        hit.roughX = wx; hit.roughY = wy; hit.s = s; hit.localX = d.x; hit.localY = d.y;
      }
    }
  }

  /**
   * ラップ終了時に1回呼ぶ。境界点と同じ「補正後姿勢列 + 記録時ローカル
   * オフセット」で再投影する。
   * @param {Array<{s:number, pose:[number,number,number]}>} samples navigator.recorder.samples
   * @param {number} yawDrift navigator.yawDrift
   * @returns {Array<{x:number, y:number}>}
   */
  finalize(samples, yawDrift) {
    if (samples.length === 0) return [];
    const poses = correctedPoseSequence(samples, yawDrift);
    return this.cones.map((c) => {
      let bi = 0, bd = Infinity;
      for (let i = 0; i < samples.length; i++) {
        const d = Math.abs(samples[i].s - c.s);
        if (d < bd) { bd = d; bi = i; }
      }
      const [x, y] = vehicleToWorld(poses[bi], c.localX, c.localY);
      return { x, y };
    });
  }
}
```

- [ ] **Step 4: `ConeRecorder`を確認する**

```js
const mod2 = await import('./js/cone_avoidance.js');
const rec = new mod2.ConeRecorder();
// 同じコーンを3回、だんだん近くで検出 (localXが小さくなる)
rec.update(10, [0, 0, 0], [{ x: 3.0, y: 1.0 }]);
rec.update(10.5, [0.5, 0, 0], [{ x: 2.0, y: 1.0 }]);
rec.update(11.0, [1.0, 0, 0], [{ x: 1.0, y: 1.0 }]);
console.log(rec.cones.length); // 期待値: 1 (同じコーンとして名寄せされる)
console.log(rec.cones[0].localX); // 期待値: 1.0 (最も近距離の検出が残る)

const samples = [
  { s: 0, pose: [0, 0, 0] },
  { s: 20, pose: [20, 0, 0] },
];
const finalized = rec.finalize(samples, 0);
console.log(finalized.length === 1, typeof finalized[0].x === 'number'); // true true
```

Expected: `1`、`1`、`true true` の順で出力される。

- [ ] **Step 5: `deflectRacelineAroundCones`を実装する**

```js
// --- 7.3: 2周目レーシングラインの回避後処理 ---
export const DEFLECT_CLEARANCE = VEHICLE_HALF_WIDTH + CONE_RADIUS + 0.20; // 0.75

/**
 * QP出力のレーシングライン点列を、記録済みコーンから離すよう局所的に
 * 押し出す。optimizeRaceline自体は変更しない後処理。
 * @param {Array<[number,number]>} points 閉曲線、順序あり
 * @param {number[]} speeds pointsと同じ長さ
 * @param {Array<{x:number, y:number}>} cones finalize()の出力
 * @returns {{points: Array<[number,number]>, speeds: number[]}}
 */
export function deflectRacelineAroundCones(points, speeds, cones) {
  const out = points.map((p) => [...p]);
  const n = out.length;
  for (const cone of cones) {
    let bestI = -1, bestD = Infinity;
    for (let i = 0; i < n; i++) {
      const d = Math.hypot(out[i][0] - cone.x, out[i][1] - cone.y);
      if (d < bestD) { bestD = d; bestI = i; }
    }
    if (bestD >= DEFLECT_CLEARANCE) continue;
    for (let k = -8; k <= 8; k++) {
      const i = ((bestI + k) % n + n) % n;
      const dx = out[i][0] - cone.x, dy = out[i][1] - cone.y;
      const dist = Math.max(Math.hypot(dx, dy), 1e-6);
      if (dist >= DEFLECT_CLEARANCE) continue;
      const push = (DEFLECT_CLEARANCE - dist) * (1 - Math.abs(k) / 9);
      out[i][0] += (dx / dist) * push;
      out[i][1] += (dy / dist) * push;
    }
  }
  return { points: out, speeds };
}

/**
 * navigator.raceline/navigator.followerを、コーン回避済みのfollowerに
 * 差し替える。navigator.state === RACINGへの遷移直後に1回だけ呼ぶ。
 * followerもracelineもLaneNavigatorの公開プロパティ (private化されて
 * いない) なので、外部から代入するだけでよい。
 * @param {import('./lane_navigator.js').LaneNavigator} navigator
 * @param {Array<{x:number, y:number}>} cones
 * @param {object} trackerParams navigator.p.tracker
 */
export function applyRacelineDeflection(navigator, cones, trackerParams) {
  const { points, speeds } = deflectRacelineAroundCones(navigator.raceline.points, navigator.raceline.speed, cones);
  navigator.follower = new RacelineFollower(points, speeds, trackerParams);
}
```

- [ ] **Step 6: `deflectRacelineAroundCones`を確認する**

```js
const mod3 = await import('./js/cone_avoidance.js');
// 半径5mの円周上に点を12個配置した「レーシングライン」
const n = 12;
const points = Array.from({ length: n }, (_, i) => {
  const a = (i / n) * 2 * Math.PI;
  return [5 * Math.cos(a), 5 * Math.sin(a)];
});
const speeds = points.map(() => 1.0);
// (5, 0)付近、つまりpoints[0]のすぐ近くにコーンを置く
const { points: out } = mod3.deflectRacelineAroundCones(points, speeds, [{ x: 5, y: 0 }]);
const distBefore = Math.hypot(points[0][0] - 5, points[0][1] - 0);
const distAfter = Math.hypot(out[0][0] - 5, out[0][1] - 0);
console.log('pushed away', distAfter > distBefore, distAfter >= mod3.DEFLECT_CLEARANCE - 1e-6);
```

Expected: `true true` が出力される（最も近い点がコーンからDEFLECT_CLEARANCE以上離れるよう押し出される）。

- [ ] **Step 7: `coneLandmarkCorrection`を実装する**

```js
// --- 7.4: 2周目、コーンランドマークによるオドメトリ補正 ---

/**
 * 検出中のコーンを記憶済みコーン地図と照合し、ズレをnavigatorに反映する。
 * navigator.state === RACINGの間、毎ティック呼ぶ。位置(x,y)のみ補正、
 * 姿勢は点ランドマーク1つからは決まらないため常に0を渡す。
 * @param {import('./lane_navigator.js').LaneNavigator} navigator
 * @param {[number, number, number]} localizerPose [localizer.x, localizer.y, localizer.yaw]
 * @param {Array<{x:number, y:number}>} detections 車体フレーム
 * @param {Array<{x:number, y:number}>} coneMapPoints finalize()の出力
 * @param {number} gate [m] この距離を超える照合は棄却する
 */
export function coneLandmarkCorrection(navigator, localizerPose, detections, coneMapPoints, gate = 1.0) {
  if (coneMapPoints.length === 0 || detections.length === 0) return;
  const mp = navigator.mapPose(localizerPose);
  let sumDx = 0, sumDy = 0, count = 0;
  for (const d of detections) {
    const [wx, wy] = vehicleToWorld(mp, d.x, d.y);
    let best = null, bestDist = Infinity;
    for (const c of coneMapPoints) {
      const dist = Math.hypot(c.x - wx, c.y - wy);
      if (dist < bestDist) { bestDist = dist; best = c; }
    }
    if (best && bestDist < gate) { sumDx += best.x - wx; sumDy += best.y - wy; count++; }
  }
  if (count > 0) navigator.applyExternalCorrection(sumDx / count, sumDy / count, 0);
}
```

- [ ] **Step 8: `coneLandmarkCorrection`を確認する**

```js
const { LaneNavigator } = await import('./js/lane_navigator.js');
const mod4 = await import('./js/cone_avoidance.js');
const nav = new LaneNavigator();
console.log(nav.corr); // [0,0,0]
// 実際には(3,0)にあるはずのコーンを、車体系で(2,0)として検出 = 1m手前にズレている
mod4.coneLandmarkCorrection(nav, [0, 0, 0], [{ x: 2, y: 0 }], [{ x: 3, y: 0 }], 2.0);
console.log(nav.corr); // corr[0]が正の値に変化しているはず (前方へ補正)
```

Expected: 2回目の`console.log`で`nav.corr[0] > 0`になる。

- [ ] **Step 9: Commit**

```bash
git add web_simulator/js/cone_avoidance.js
git commit -m "$(cat <<'EOF'
feat(web_simulator): add cone avoidance logic (reactive nudge, mapping,
raceline deflection, landmark drift correction)

Pure functions that consume LaneNavigator's public surface (raceline,
follower, mapPose(), applyExternalCorrection()) without modifying the
ported algorithm itself.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 9: `simulator.js`への統合配線 + HUD状態表示

**Files:**
- Modify: `web_simulator/js/simulator.js`（Task 5で追加したimportの近くに追加import、`stepNavigator`/`runPerception`/HUD更新）
- Modify: `web_simulator/index.html:316-335`（自動運転タブに検出状態表示を追加）, `web_simulator/index.html:337-`（詳細タブにスリップ表示を追加）
- Modify: `web_simulator/README.md`（ファイル構成・新機能の説明を追記）

**Interfaces:**
- Consumes: Task 1, 2, 3, 4, 5, 7, 8 のすべて

- [ ] **Step 1: コーン検知器のロードと検知呼び出しを配線する**

`web_simulator/js/simulator.js`の冒頭importに追加:

```js
import { ConeDetector } from './cone_detector.js';
import { reactiveAvoid, ConeRecorder, applyRacelineDeflection, coneLandmarkCorrection } from './cone_avoidance.js';
```

既存の`yolopDetector`/`ufldDetector`が定義されている箇所（785行目付近）の近くに追加:

```js
const coneDetector = new ConeDetector();
const CONE_ONNX_URL = 'models/cone.onnx';
let coneLoadPromise = null;
let latestConeDetections = [];
const coneRecorder = new ConeRecorder();
let prevReactiveBias = 0;
let previousNavState = null;
const coneStatusEl = document.getElementById('oit-cone-status');

function ensureConeDetectorLoading() {
  if (coneDetector.session || coneLoadPromise) return;
  coneLoadPromise = coneDetector.load(CONE_ONNX_URL, MODEL_WASM_DIR)
    .then(() => { coneStatusEl.textContent = 'コーン検知: 読込済'; })
    .catch((err) => {
      console.warn('ConeDetector load failed (models/cone.onnx missing?)', err);
      coneStatusEl.textContent = 'コーン検知: 読込エラー (models/cone.onnx を生成してください)';
      coneLoadPromise = null;
    });
}
ensureConeDetectorLoading();
```

`MODEL_WASM_DIR`は既存の`js/simulator.js`に定義済み（YOLOP/UFLD読み込みで使っているもの）をそのまま使う。

- [ ] **Step 2: 検知を`runPerception`に組み込む**

`runPerception(now)`関数（1301行目付近）の中、`lineTracker.update(fits)`の直後（1328行目付近）に追加:

```js
    if (detectorMode !== 'ros2' && coneDetector.session) {
      try {
        latestConeDetections = await coneDetector.infer(captureCanvas);
      } catch (err) {
        console.error('cone detector inference error', err);
        latestConeDetections = [];
      }
    } else {
      latestConeDetections = [];
    }
    if (laneNavigator.state === MAPPING) {
      coneRecorder.update(localizer.s, [localizer.x, localizer.y, localizer.yaw], latestConeDetections);
    }
```

- [ ] **Step 3: 反応的回避を`stepNavigator`に組み込む**

`stepNavigator(tracked, now)`関数（1263-1279行目）を次のように変更する:

```js
function stepNavigator(tracked, now = performance.now() / 1000) {
  const dt = lastNavStepTime === null ? 0 : Math.min(now - lastNavStepTime, 0.5);
  lastNavStepTime = now;
  const pose = [localizer.x, localizer.y, localizer.yaw];
  const rawCmd = laneNavigator.step(now, dt, pose, localizer.v, localizer.omega, localizer.s, tracked);
  const avoided = reactiveAvoid(rawCmd, latestConeDetections, prevReactiveBias, dt);
  prevReactiveBias = avoided.bias;
  const cmd = { v: avoided.v, omega: avoided.omega };

  // MAPPING -> RACING遷移を検知したら、1回だけレーシングラインをコーン回避
  // 後処理版に差し替え、コーン地図を確定する。
  if (previousNavState !== RACING && laneNavigator.state === RACING && laneNavigator.raceline) {
    const finalizedCones = coneRecorder.finalize(laneNavigator.recorder.samples, laneNavigator.yawDrift);
    applyRacelineDeflection(laneNavigator, finalizedCones, laneNavigator.p.tracker);
    window.__sim.coneMapPoints = finalizedCones; // デバッグ確認用
  }
  previousNavState = laneNavigator.state;

  if (laneNavigator.state === RACING && window.__sim.coneMapPoints && window.__sim.coneMapPoints.length) {
    coneLandmarkCorrection(laneNavigator, pose, latestConeDetections, window.__sim.coneMapPoints);
  }

  latestAutonomousCmd = { v: cmd.v, omega: cmd.omega };
  twistMux.update('mpc', cmd.v, cmd.omega, performance.now());
  if (autonomousCmdVelTopic) {
    autonomousCmdVelTopic.publish(
      new ROSLIB.Message({ linear: { x: cmd.v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: cmd.omega } })
    );
  }
  const st = laneNavigator.status();
  showNavStatus(st);
  if (laneTrackerStatusTopic) laneTrackerStatusTopic.publish(new ROSLIB.Message({ data: JSON.stringify(st) }));
  return cmd;
}
```

`RACING`は既にこのファイルの冒頭で`lane_navigator.js`からimportされている（14行目付近の`NAVIGATOR_PARAMS, RACELINE_PARAMS, TRACKER_PARAMS, MAPPING, RACING, ROLES`のimportにある）ので追加のimportは不要。

`resetNavigation()`関数（853行目付近）に、コーン記憶のリセットも追加する:

```js
  coneRecorder.cones.length = 0;
  prevReactiveBias = 0;
  previousNavState = null;
  window.__sim.coneMapPoints = [];
```

- [ ] **Step 4: HUDにコーン検知・スリップ表示を追加する**

`web_simulator/index.html`の自動運転タブ（321行目、`<dt>検出器</dt><dd id="oit-detector-status">-</dd>`の直後）に追加:

```html
        <dt>コーン検知</dt><dd id="oit-cone-status">-</dd>
```

詳細タブのOdometryセクション（340行目付近、`<dt>角速度 Z</dt><dd id="odom-wz">0.00 rad/s</dd>`の直後）に追加:

```html
        <dt>スリップ L/R</dt><dd><span id="slip-l">0.0%</span> / <span id="slip-r">0.0%</span></dd>
```

`simulator.js`のHUD更新ループ（既存の`odomVxVal`/`odomVyVal`/`odomWzVal`を毎フレーム更新している箇所、`animate()`内）に追加:

```js
const slipLEl = document.getElementById('slip-l');
const slipREl = document.getElementById('slip-r');
```

（トップレベルの他のDOM取得と同じ場所に追加し）、`animate()`内の既存のHUD数値更新箇所に:

```js
    slipLEl.textContent = `${(physics.slipL * 100).toFixed(1)}%`;
    slipREl.textContent = `${(physics.slipR * 100).toFixed(1)}%`;
```

- [ ] **Step 5: `window.__sim`デバッグオブジェクトにコーン関連を追加する**

既存の`Object.assign(window.__sim, {...})`（またはそれに相当する初期化）に追加:

```js
  coneDetector, coneRecorder, latestConeDetections: () => latestConeDetections, coneMapPoints: [],
```

- [ ] **Step 6: `models/cone.onnx`なしでもエラーにならないことを確認する**

```bash
python3 web_simulator/serve.py 8010
```

`web_simulator/models/cone.onnx`が存在しない状態（Task 6のスクリプトをまだ実行していない状態）でブラウザを開き:

```js
await new Promise((r) => setTimeout(r, 2000));
console.log(document.getElementById('oit-cone-status').textContent);
```

Expected: `"コーン検知: 読込エラー (models/cone.onnx を生成してください)"` と表示され、コンソールに未処理の例外（uncaught exception）が出ていないこと。自動運転をONにして走行させても、コーン検知以外の機能（白線追従・衝突判定）が通常通り動作すること。

- [ ] **Step 7: `models/cone.onnx`ありでコーンを避けることを確認する（`cone.onnx`が用意できる場合のみ）**

`models/cone.onnx`が生成済みであれば、次を確認する: HUD「コーン配置」をONにし、コースの中央線付近にコーンを1本置く。自動運転をONにし、車両がそのコーン付近に近づくコマンドを`fastForward`等で進める。コンソールで:

```js
const before = window.__sim.coneEditor.cones.length;
console.log('placed', before);
// 自動運転ON、コーン付近を通過するまで進める (既存のfastForward(sec)を使う)
window.__sim.fastForward(60);
console.log('slip L/R', window.__sim.physics.slipL, window.__sim.physics.slipR);
console.log('cone map points after lap1', window.__sim.coneMapPoints.length);
```

Expected: `coneMapPoints.length`が1以上（1周目でコーンが記憶されている）。HUDの「接触中」表示が、そのコーンの通過中に継続的にはON（衝突しっぱなし）にならないこと（多少接触があっても、反応的回避が効いて`Cone`をすり抜けずに通過できていることを目視・HUDログで確認する）。

`cone.onnx`を用意できない場合はこのステップをスキップしてよい（Task 6でスクリプトの構文のみ確認済み、Step 6のフォールバック確認で代替する）。

- [ ] **Step 8: READMEを更新する**

`web_simulator/README.md`のファイル構成の節に、以下を追記する（既存の`js/lane_model_detector.js`等の説明が並ぶリストと同じ形式）:

```markdown
│   ├── cone_props.js               コーンの3Dモデル(models/cone.glb)読み込み・当たり判定
│   ├── cone_editor.js              コーンのクリック配置・ドラッグ移動・削除 (localStorage永続化)
│   ├── cone_detector.js            cone.onnx によるコーン検知 (バウンディングボックス -> 地面座標)
│   ├── cone_avoidance.js           コーン回避 (反応的ナッジ・1周目記憶・2周目レーシングライン回避・ランドマーク補正)
```

「コース」節の近くに新しい節「コーン配置・検知・回避」「スリップ誤差モデル」を追加し、docs/superpowers/specs/2026-09-22-cone-avoidance-design.mdの2〜7章の要点（配置方法、検知の仕組み、反応的回避と2周目後処理の違い、スリップモデルの意図）を日本語で簡潔にまとめる。

- [ ] **Step 9: Commit**

```bash
git add web_simulator/js/simulator.js web_simulator/index.html web_simulator/README.md
git commit -m "$(cat <<'EOF'
feat(web_simulator): wire cone detection + avoidance into the sim loop

Cone detector runs every perception tick; reactive avoidance biases
the autonomous command in both MAPPING and RACING; lap-1 cone
recordings deflect the lap-2 raceline and correct odometry drift via
cone landmarks. Falls back gracefully when models/cone.onnx is absent.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## 完了確認（全タスク後、1回通しで実施）

- [ ] **Step 1: フルシナリオを1回通しで確認する**

```bash
python3 web_simulator/serve.py 8010
```

ブラウザで`index.html`を開き:
1. 「コーン配置」をONにし、コース上に2〜3本コーンを配置する。
2. ページをリロードし、コーンが復元されていることを確認する。
3. 自動運転をONにする。
4. `window.__sim.fastForward(180)`で1周目〜2周目序盤まで進める。
5. HUD「詳細」タブのスリップ表示が0%でないことを確認する。
6. `window.__sim.physics.x/y`と「詳細」タブのOdometry位置がわずかにズレていることを確認する。
7. `window.__sim.coneMapPoints.length`がコーン本数と概ね一致することを確認する（誤検出・見逃しで多少前後してもよい）。
8. コンソールに未処理のエラーが出ていないことを確認する。

Expected: 1〜8すべてが期待通り。

- [ ] **Step 2: 最終レビュー観点の自己チェック**

- `js/lane_navigator.js`の差分が、`correctedPoseSequence`のexport（動作不変）と`applyExternalCorrection`の追加のみであること（`git diff` `web_simulator/js/lane_navigator.js`を確認）。
- `optimizeRaceline`/`_mapMatching`/`_stepMapping`/`_stepRacing`に差分がないこと。
