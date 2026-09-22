# コーン配置・検知・回避 設計書

**日付**: 2026-09-22
**スコープ**: `web_simulator` のみ（実機ROSノードは別タスク）

## 1. 背景・目的

`web_simulator` に、ユーザーがコースにコーンを自由配置し、それを `models/cone.pt`（YOLO）で実際のカメラ画像から検知し、自律走行中に回避する挙動を追加する。あわせて、これまでモデル化されていなかった車輪スリップ誤差（CLAUDE.md記載の「8%程度」）をシミュレータの機体に搭載し、カメラによるコーン距離測定を使ってオドメトリのドリフトを補正する仕組みも入れる。

実機側（`src/`）は今回変更しない。`models/cone.pt` を消費するコードは今回が最初だが、ROSノードとしては作らず、ブラウザ用ONNXへの変換スクリプトのみ `src/oit_navigation/` に追加する（既存の `export_onnx_web.py`/UFLD書き出しと同じ「ブラウザ用アセットを作るツール」という位置づけで、ROSノードではない）。

## 2. 全体アーキテクチャ

```
[コーン配置エディタ]  --localStorage-->  [コーンインスタンス管理]  --3Dモデル・コライダー-->  [collision.js]
        (cone_editor.js)                    (cone_props.js)

[captureCanvas] --> [cone_detector.js: cone.onnx推論] --> ボウンディングボックス列
                                                                |
                                                     projectToGround (lane_navigator.jsの既存関数)
                                                                |
                                                        車体フレーム上のコーン位置
                                                                |
                                              +----------------+-----------------+
                                              |                                  |
                                   [反応的回避 (毎フレーム)]          [1周目: コーン記憶 (ConeRecorder)]
                                   navigator.step()の出力を                      |
                                   外側で補正 (cone_avoidance.js)      ラップ終了時に境界点と同じ
                                              |                        ドリフト補正・ループクロージャ
                                              |                                  |
                                              |                        [2周目開始: レーシングライン
                                              |                         へのコーン回避後処理]
                                              |                                  |
                                              +----------------+-----------------+
                                                                |
                                                [2周目: コーンランドマーク照合による
                                                 オドメトリドリフト補正]

[vehicle_physics.js: 左右輪スリップモデル] --> CAN RPM配信 / integrateLocalizer (オドメトリ推定)
                                                (真の物理位置 physics.x/y/yaw は不変)
```

**設計原則**: `js/lane_navigator.js` は実機の `src/oit_navigation/oit_navigation/lane_nav/*.py` を1:1移植したものであり、ファイル冒頭のコメントで「このシミュレータは実機とまったく同じアルゴリズムを実行する」と明記されている。今回は実機側を変更しないため、この移植の忠実性を壊さないよう、回避・補正ロジックの**計算本体**はすべて新規ファイル（`cone_avoidance.js`）に置き、`lane_navigator.js`への変更は次の最小限のみとする:

1. `LaneNavigator.applyExternalCorrection(dx, dy, dyaw)` という公開メソッドを1つ追加する（`_mapMatching()`の末尾がやっている `this.corr` の更新と同じ計算を、外部から呼べる形にしただけ。アルゴリズムの変更ではない）。
2. `course_map.py` 移植部分の内部関数 `correctYawDrift` を、姿勢列の計算部分と再投影部分に分割し、姿勢列の計算部分を `export function correctedPoseSequence(samples, yawDrift)` として切り出す（**動作は変えず**、`buildCourseMap`が計算する境界点は変更前と完全に同一になる）。境界点の補正は「世界座標を直接回転する」のではなく「車体ローカルオフセット＋補正後の姿勢列から再投影する」方式なので、コーン記憶側もこの`correctedPoseSequence`をそのまま再利用して同じ方式で再投影する（7.2節）。

## 3. コーン配置エディタ (`js/cone_editor.js`)

- HUDに「コーン配置: OFF/ON」トグルを追加。ONの間、3Dビュー上の操作:
  - **空いた場所をクリック**: レイキャストでasphalt/ground メッシュとの交点を取り、その地点にコーンを1本追加。
  - **既存コーンをドラッグ**（mousedown→moveがしきい値5pxを超えてから→mouseup）: つまんだコーンを新しい地面座標に移動。ドラッグ中はプレビュー表示。
  - **既存コーンをクリック**（ドラッグなしで離す）: そのコーンを削除。
  - 「全消去」ボタン。
- 本数無制限。中央線からの距離やレーン内かどうかの制約は課さない。
- 状態は `[{id, x, y}]` の配列として `localStorage`（キー: `aiformula_cones_v1`）に保存し、ページ読み込み時に復元。
- 公開インターフェース: `createConeEditor({scene, groundMesh, onChange})` → `{ enable(), disable(), cones: [{id,x,y}], addCone(x,y), removeCone(id), moveCone(id,x,y), clearAll() }`。`onChange`はコーン配列が変わるたびに呼ばれ、`simulator.js`側でコライダー・3Dモデルを再構築する。

## 4. コーンのモデル・当たり判定 (`js/cone_props.js`)

`models/cone.glb` を解析した実測値（`Cone_Base`/`Cone_Body`/`Cone_Flange`の3メッシュ、ルートnode scale 0.001・子nodeそれぞれscale 10、course.glbやMyLapsゲートの円錐コライダーと同じ実寸系）:

- 底面半径: 0.15m（`Cone_Base`のXZ範囲 ±15 × 0.01）
- 高さ: 0.45m（`Cone_Body`のY最大値 45 × 0.01）
- 実測値は`MYLAPS_COLLIDERS`のコーン半径（0.15m）と一致しており、同じ規格の450mm三角コーン。

```js
export const CONE_SCALE = 0.01; // 0.001(root) x 10(children)
export const CONE_RADIUS = 0.15; // [m] 当たり判定・回避マージン計算の両方で使う
export const CONE_HEIGHT = 0.45; // [m] 参考値（当たり判定は円のみ、高さは使わない）
```

- `addCone(parent, setPose, {x, y, id})`: `models/cone.glb`をGLTFLoaderでロードし（course.glbと同じ読み込み関数を再利用）、`rosRoot`の子として配置。同じモデルインスタンスをコーンの数だけ複製（GLTFLoaderの`clone()`または`SkeletonUtils`不要の単純ジオメトリ共有）。
- `coneWorldColliders(cones)`: 各コーンについて `{x, y, r: CONE_RADIUS}` を返し、`collision.js`の`resolveCollisions()`の`obstacles`配列に、既存のMyLapsコライダーと一緒に渡す。
- コーンエディタの`onChange`のたびに、シーン上のコーンオブジェクトを全部作り直す（数が多くても数十本程度なのでシンプルな全再構築で十分）。

## 5. カメラ画像からのコーン検知 (`js/cone_detector.js`)

### 5.1 ONNXへの変換

新規 `src/oit_navigation/oit_navigation/export_cone_onnx.py`（ROSノードではなく、`export_onnx_web.py`と同じブラウザ用アセット書き出しツール）:

```python
#!/usr/bin/env python3
"""export_cone_onnx.py - cone.pt (Ultralytics YOLO) を web_simulator 用ONNXへ変換する。

Usage:
    python3 export_cone_onnx.py \
        --weights /aiformula_machine/models/cone.pt \
        --output /aiformula_machine/web_simulator/models/cone.onnx
"""
import argparse
from ultralytics import YOLO


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", default="/aiformula_machine/models/cone.pt")
    parser.add_argument("--output", default="/aiformula_machine/web_simulator/models/cone.onnx")
    parser.add_argument("--size", type=int, default=640)
    parsed = parser.parse_args(args=args)

    model = YOLO(parsed.weights)
    exported = model.export(format="onnx", imgsz=parsed.size, opset=12, simplify=True)
    import shutil
    shutil.move(exported, parsed.output)
    print(f"[export_cone_onnx] Wrote ONNX graph: {parsed.output}")


if __name__ == "__main__":
    main()
```

`web_simulator/README.md`に、UFLDと同様「git管理外、生成手順」として追記する。

### 5.2 ブラウザ側推論 (`ConeDetector`クラス、`lane_model_detector.js`と同じ構成)

- 入力: `captureCanvas`（オンボードカメラ描画、YOLOP/UFLDと共有）。
- 前処理: レターボックス方式で640x640にリサイズ（Ultralyticsのデフォルト書き出しがレターボックス前提のため、YOLOPのcrop_bottomとは異なる）。
- 出力デコード: Ultralytics YOLO ONNXの標準出力形状 `[1, 4+num_classes, 8400]` を読み、信頼度閾値0.4、IoU 0.45でNMSを行い `[{x1,y1,x2,y2,conf,cls}]` を返す。
- WebGPU→WASMフォールバックは`ModelLaneDetector`と同じ実装を流用。

### 5.3 距離・横位置の推定

信号機検知(`traffic_light_distance_node.py`)は対象が地面から浮いているため占有率からの逆算が必要だが、コーンは接地しているため、`lane_navigator.js`の`projectToGround()`（白線検知と同じカメラモデル・地面投影）にバウンディングボックスの**下辺中央ピクセル**を渡すだけで車体フレームの(x, y)が直接求まる。新しい距離推定式を書く必要はない。

- 有効範囲: `0.3m < x < 8.0m`、`|y| < 3.0m`（それ以外は誤検出/範囲外として無視）。

## 6. スリップ誤差モデル (`js/vehicle_physics.js`)

現状確認: `wheelSpeeds()`は真の車輪速度をそのまま返し、`simulator.js`の`integrateLocalizer(physics.v, physics.omega, dt)`も真値をノータッチで積分している。CAN配信(`publishVehicleInfoCan`)も同様。つまり現在オドメトリ誤差はゼロ。

### 6.1 モデル

左右輪独立に、時定数付きランダムウォークで±8%以内のスリップ率を持たせる:

```js
// vehicle_physics.js に追加
const SLIP_TAU_S = 2.0;       // [s] 時定数
const SLIP_NOISE = 0.05;      // [1/sqrt(s)] ノイズ強度（時定数と合わせて定常分散が概ね±8%に収まるよう調整）
const SLIP_MAX = 0.08;        // ±8%にクランプ

class VehiclePhysics {
  constructor() {
    ...
    this.slipL = 0;
    this.slipR = 0;
  }

  _stepSlip(dt) {
    const randn = () => { // Box-Muller
      const u1 = Math.max(Math.random(), 1e-9), u2 = Math.random();
      return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
    };
    this.slipL += (-this.slipL / SLIP_TAU_S + SLIP_NOISE * randn()) * dt;
    this.slipR += (-this.slipR / SLIP_TAU_S + SLIP_NOISE * randn()) * dt;
    this.slipL = clamp(this.slipL, -SLIP_MAX, SLIP_MAX);
    this.slipR = clamp(this.slipR, -SLIP_MAX, SLIP_MAX);
  }

  // 真の車輪速度 (物理演算・当たり判定・描画に使う、従来通り)
  wheelSpeeds() { ... }  // 変更なし

  // 計測される車輪速度 (CAN配信・オドメトリ推定だけが使う、スリップ込み)
  measuredWheelSpeeds() {
    const { left, right } = this.wheelSpeeds();
    return { left: left * (1 + this.slipL), right: right * (1 + this.slipR) };
  }
}
```

`step()`/`stepAutonomous()`の末尾で`this._stepSlip(dt)`を呼ぶ。

### 6.2 配線変更 (`simulator.js`)

- `publishVehicleInfoCan()`: `physics.wheelSpeeds()` → `physics.measuredWheelSpeeds()`
- `animate()`内の `integrateLocalizer(physics.v, physics.omega, dt)`:
  `measuredWheelSpeeds()`から`vMeas = (left+right)/2`, `omegaMeas = (right-left)/VEHICLE.track`を計算し、それを渡す形に変更。
- 車輪スピンアニメーション(`wheelLeftSpin`等)は真の`wheelSpeeds()`のまま（見た目は物理的な回転そのものなので変更しない）。
- HUDに現在のスリップ率（`physics.slipL`, `physics.slipR`）を表示する項目を追加（デバッグ用）。

これにより、`physics.x/y/yaw`（真の位置、カメラ描画・当たり判定・HUD「真値」表示に使用）と`localizer.x/y/yaw`（`LaneNavigator`に渡る推定位置）が走行中に乖離していく。

## 7. 回避レイヤー (`js/cone_avoidance.js`)

### 7.1 反応的回避（毎フレーム、MAPPING/RACING共通）

`navigator.step()`が返す`{v, omega}`を、以下の補正を加えた上で`physics.stepAutonomous()`に渡す(`simulator.js`側の呼び出し変更)。

```js
import { VEHICLE_HALF_WIDTH } from './collision.js';
import { CONE_RADIUS } from './cone_props.js';

const REACT_MARGIN = 0.15;             // [m] 追加安全マージン
const REACT_CLEARANCE = VEHICLE_HALF_WIDTH + CONE_RADIUS + REACT_MARGIN; // 0.70m
const REACT_LOOKAHEAD_X = 4.0;         // [m] この前方距離より遠いコーンは無視
const REACT_MIN_X = 0.3;
const REACT_GAIN = 1.5;
const REACT_MAX_OMEGA_BIAS = 0.6;      // [rad/s]
const REACT_SLOW_X = 1.5;              // [m] これより近いと減速
const REACT_SLOW_V = 0.5;              // [m/s] 減速後の上限速度

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
  const bias = rateLimit(prevBias, targetBias, maxRate, dt); // lane_navigator.jsのrateLimitを再利用
  return {
    v: closeSlow ? Math.min(cmd.v, REACT_SLOW_V) : cmd.v,
    omega: cmd.omega + bias,
    bias,
  };
}
```

`rateLimit`は`lane_navigator.js`が既にexportしている純粋関数をそのままimportして再利用する(重複実装しない)。

### 7.2 1周目: コーン記憶 (`ConeRecorder`)

境界点の補正（`correctedPoseSequence`、2章参照）は「世界座標を直接回転・平行移動する」のではなく、「補正後の姿勢列 × 記録時の車体ローカルオフセット」で世界座標を**再投影**する方式である。コーンも同じ方式に乗せるため、`ConeRecorder`は世界座標ではなく「検出時の`s`・車体ローカルオフセット(`localX, localY`)」を保持する。1つのコーンは接近中に何十フレームも検出されるため、**最も近距離（`localX`最小）で捉えた1回分**だけを代表値として残す（遠距離ほど`projectToGround`の角度分解能が粗くなるため）。

```js
export class ConeRecorder {
  constructor(gateRadius = 0.6) { this.cones = []; this.gateRadius = gateRadius; }

  // s: localizer.s, pose: [localizer.x, localizer.y, localizer.yaw]
  update(s, pose, detections) {
    for (const d of detections) {
      const [wx, wy] = vehicleToWorld(pose, d.x, d.y); // 名寄せ用のラフな世界座標(未補正)
      const hit = this.cones.find(c => Math.hypot(c.roughX - wx, c.roughY - wy) < this.gateRadius);
      if (!hit) {
        this.cones.push({ roughX: wx, roughY: wy, s, localX: d.x, localY: d.y });
      } else if (d.x < hit.localX) {
        // より近距離の検出で置き換え(roughXYは名寄せ用に更新し続ける)
        hit.roughX = wx; hit.roughY = wy; hit.s = s; hit.localX = d.x; hit.localY = d.y;
      }
    }
  }

  // buildCourseMap呼び出し直後、samples/yawDriftが確定してから1回呼ぶ。
  // 各コーンのsに最も近い記録サンプルの「補正後の姿勢」で再投影する
  // (samplesの間隔は数m単位なので最近傍サンプルで十分、線形補間はしない)。
  finalize(samples, yawDrift) {
    const poses = correctedPoseSequence(samples, yawDrift); // lane_navigator.jsからimport
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

`simulator.js`は`LaneNavigator.state === MAPPING`の間、毎ティック`coneRecorder.update(localizer.s, [localizer.x, localizer.y, localizer.yaw], detections)`を呼ぶ。ラップ終了時（`navigator.state`がMAPPING→OPTIMIZINGへ変わったことを検知した次のティック）、`coneRecorder.finalize(navigator.recorder.samples, navigator.yawDrift)`を呼び、補正済みのコーン世界座標一覧を得る。これを7.3節・7.4節で使う。

### 7.3 2周目: レーシングラインのコーン回避後処理

`navigator.state`がRACINGに変わった直後、`navigator.follower`（`RacelineFollower`インスタンス、公開プロパティ）を、コーン回避済みの経路で作り直した`RacelineFollower`に**外部から差し替える**（`LaneNavigator`側の変更は不要、`follower`は元からprivateではない公開プロパティ）。

```js
import { RacelineFollower } from './lane_navigator.js';

const DEFLECT_CLEARANCE = VEHICLE_HALF_WIDTH + CONE_RADIUS + 0.20; // 0.75m

export function deflectRacelineAroundCones(points, speeds, cones, boundaryHalfWidth) {
  const out = points.map((p) => [...p]);
  const n = out.length;
  for (const cone of cones) {
    let bestI = -1, bestD = Infinity;
    for (let i = 0; i < n; i++) {
      const d = Math.hypot(out[i][0] - cone.x, out[i][1] - cone.y);
      if (d < bestD) { bestD = d; bestI = i; }
    }
    if (bestD >= DEFLECT_CLEARANCE) continue;
    // 前後数点をコーンから離す (影響範囲: 前後 ±8点、Catmull-Romで滑らかに再接続)
    for (let k = -8; k <= 8; k++) {
      const i = ((bestI + k) % n + n) % n;
      const dx = out[i][0] - cone.x, dy = out[i][1] - cone.y;
      const dist = Math.max(Math.hypot(dx, dy), 1e-6);
      if (dist >= DEFLECT_CLEARANCE) continue;
      const push = (DEFLECT_CLEARANCE - dist) * (1 - Math.abs(k) / 9); // 減衰
      out[i][0] += (dx / dist) * push;
      out[i][1] += (dy / dist) * push;
    }
  }
  return { points: out, speeds };
}
```

生成した`{points, speeds}`から`new RacelineFollower(points, speeds, tracker_params)`を作り、`navigator.follower = newFollower`で差し替える。`RacelineFollower`のコンストラクタが内部で`densifyClosed`（Catmull-Rom）を呼ぶため、上のpush処理で生じた折れ線も自動的に滑らかになる。コース境界を超えないためのクランプ（`boundaryHalfWidth`引数）は今回は簡略化し、後処理後の点が左右境界ポリラインの内側に収まっているかだけ確認し、超えていれば7.1節の反応的回避に任せる（ログにも警告を出す）。

### 7.4 2周目: コーンランドマークによるオドメトリ補正

`LaneNavigator`に追加する公開メソッド:

```js
// lane_navigator.js に追加。_mapMatching()末尾の corr 更新と同じ演算を外から呼べるようにしただけ。
applyExternalCorrection(dx, dy, dyaw, damping = 0.15) {
  const delta = [dx * damping, dy * damping, dyaw * damping];
  const mp = this.corr;
  // _mapMatching()の末尾と同じ合成
  const nw = [mp[0] + delta[0], mp[1] + delta[1], mp[2] + delta[2]];
  ...(既存の合成コードと同一)...
}
```

`cone_avoidance.js`側（RACING中、毎ティック）:

```js
export function coneLandmarkCorrection(navigator, detections, coneMapPoints, gate = 1.0) {
  const mp = navigator.mapPose([localizer.x, localizer.y, localizer.yaw]); // mapPoseは既に公開メソッド
  let sumDx = 0, sumDy = 0, count = 0;
  for (const d of detections) {
    const [wx, wy] = vehicleToWorld(mp, d.x, d.y);
    const nearest = coneMapPoints.reduce((best, c) => {
      const dist = Math.hypot(c.x - wx, c.y - wy);
      return dist < best.dist ? { c, dist } : best;
    }, { c: null, dist: Infinity });
    if (nearest.c && nearest.dist < gate) {
      sumDx += nearest.c.x - wx; sumDy += nearest.c.y - wy; count++;
    }
  }
  if (count > 0) navigator.applyExternalCorrection(sumDx / count, sumDy / count, 0);
}
```

これは`_mapMatching()`が白線で行っている「検出位置 vs 地図位置」の照合と同じ考え方を、コーンという点ランドマークに対して行うもの。位置(x, y)のみ補正し、姿勢(yaw)はコーン1点からは決められないため今回は0のまま（`_mapMatching`が線の傾きから姿勢を求めているのとは異なり、点ランドマークなので姿勢情報は得られない）。

## 8. HUD・README変更

- HUDに「コーン配置: OFF/ON」トグル、「全消去」ボタン、コーン検知ステータス（cone.onnx読み込み状態）、スリップ率表示を追加。
- `web_simulator/README.md`:
  - ファイル構成に`cone_editor.js`/`cone_props.js`/`cone_detector.js`/`cone_avoidance.js`を追記。
  - 「コーン配置」「コーン検知・回避」「スリップ誤差モデル」の節を新設。
  - `cone.onnx`の生成コマンド（`ros2 run oit_navigation export_cone_onnx`）を追記。

## 9. 既知の制約・スコープ外

- 実機ROSノード（`cone_distance_node.py`相当）は作らない。将来作る際は、今回の`projectToGround`ベースの距離推定・地面接地前提の設計をPython側にも移植することを推奨する。
- `optimizeRaceline`自体（QPの制約）にコーンを組み込む案（設計案B）は不採用。7.3節の後処理は「最適経路」ではなく「危険域からの局所退避＋平滑化」であり、コーンが密集した区間では最適性が保証されない。
- コーンランドマーク補正はRACING（2周目以降）のみ有効。MAPPING中（1周目）はまだ地図が存在しないため補正できない（`_mapMatching`と同じ制約）。
- コーン同士やコーンとMyLapsゲートが近接している場合の後処理の相互作用（回避方向の競合）は簡易実装であり、極端な密集配置では最適な迂回にならない可能性がある。
