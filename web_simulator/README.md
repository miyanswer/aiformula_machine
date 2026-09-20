# AIFormula Web Simulator

Three.js + rosbridge を用いた、WASD キーボード操作の Web 走行シミュレータです。
車体モデルは [`vehicles/sample_vehicle`](../vehicles/sample_vehicle) の xacro/メッシュ（`AIF_body.dae`, `tire.dae`）をそのまま読み込んで表示します。

## 起動〜動作確認の手順

シミュレータ単体（ブラウザ内の物理シミュレーションのみ）ならステップ1だけで動きます。
ROS 2 側に `cmd_vel` を流したい場合はステップ2〜4も行ってください。

### 1. Web シミュレータを起動する

このディレクトリだけをサーバーのルートにすると、`../vehicles/...` のメッシュ参照が
ドキュメントルートの外に出てしまい読み込めません。**必ずリポジトリのルートから**
静的サーバーを起動してください。

```bash
cd /Users/miyanswer/aiformula_machine
python3 web_simulator/serve.py 8000
```

（プレーンな `python3 -m http.server` ではなく [`web_simulator/serve.py`](serve.py) を
使う理由: YOLOP / UFLD 検出モードの onnxruntime-web が要求する COOP/COEP
ヘッダーを付与するためです。詳しくは後述の「実装上の注意」を参照してください。）

ブラウザで `http://localhost:8000/web_simulator/` を開きます。車体モデルが表示され、
WASD で動かせれば起動成功です（この時点では rosbridge 未接続でも問題ありません）。

Three.js / roslib.js はオフライン環境でも動くよう `vendor/` 配下にローカル同梱済みです
（追加のビルドや `npm install` は不要）。

**対応ブラウザ**: Chrome/Edge に加えて **Firefox でも動作確認済み**です
（WebGL描画、ONNXモデルによる白線検出、HUDのタブ切替・折りたたみ・ドラッグ移動、
いずれもエラーなし）。ONNX モデルの検出モードは WebGPU が使えない環境（`navigator.gpu` が
ブロックリスト等で無効な場合を含む）では自動的に CPU の WASM 推論にフォールバックするため、
WebGPU 非対応・無効なブラウザでも動作は継続します（推論速度は GPU 実行時より低下します）。

### 2. rosbridge_server を起動する

`rosbridge_server` は Dockerfile に標準搭載されており、ホスト側のターミナルから `make rosbridge` を実行するだけで起動できます（未インストールの環境でも自動検知してインストール・起動されます）。

```bash
make rosbridge
```

またはコンテナ内で直接起動する場合：

```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

以下のようなログが出て、WebSocket サーバーがポート `9090` で待ち受けていれば成功です。

```
[INFO] [rosbridge_websocket]: Rosbridge WebSocket server started on port 9090
```

別ターミナルで待受状態も確認できます。

```bash
ss -ltnp | grep 9090   # LISTEN していること
```

`compose.yaml` に `9090:9090` のマッピングが追加されているため、ホスト側のブラウザから `ws://localhost:9090` で直接接続できます。
静的サーバーはホスト側（または `make sim` / `make rosbridge`）で起動して `http://localhost:8000` をブラウザで開いてください。

### 3. シミュレータを rosbridge に接続する

Web シミュレータの HUD 左上で以下を設定し、「接続」ボタンを押します。

- **rosbridge URL**: 既定値 `ws://localhost:9090`（コンテナ越しにアクセスする場合は
  ホスト名/IPを書き換えてください）
- **cmd_vel topic**: 既定値 `/aiformula_control/gamepad/cmd_vel`
  （`launchers/sample_launchers/launch/twist_mux.launch.py` が実際に購読している
  gamepad 入力トピック。twist_mux の priority は 150。
  `control/twist_mux/config/twist_mux_topics.yaml` の `keyboard`（`key_vel`）経由で
  twist_mux を単体起動している場合は、ここを `key_vel` に変更してください）

ステータスが緑の「接続済み」になれば成功です。接続した時点で、停止中でも
`geometry_msgs/msg/Twist` を 10Hz で publish し続けます（速度 0 の Twist が流れるので、
twist_mux 側のタイムアウトでロックされることもありません）。

**cmd_vel タイムアウト（0.3秒）のシミュレーション**: `twist_mux`
はこの入力ソースを 0.3 秒（`topic_list.yaml` の gamepad/keyboard timeout）受信できないと
タイムアウトとして扱い、実車はその入力からは速度 0 を受け取ります。本シミュレータでも
同じ 0.3 秒のウォッチドッグを**車体そのものの挙動**に適用しています。rosbridge が
切断された（あるいは何らかの理由で publish が 0.3 秒以上止まった）場合、以後 WASD を
押し続けていても新しい入力として扱われず、通常の惰性走行と同じ減速で自動的に停止します
（[js/simulator.js](js/simulator.js) の `CMD_VEL_TIMEOUT_SEC`）。接続中はブラウザ内蔵の
描画ループが cmd_vel を継続的に publish しているため、この機構は通常は作動しません
（切断時など、実際にタイムアウトが起きる状況でのみ発火します）。

### 4. トピックが流れているか確認する

rosbridge が起動しているマシン（コンテナ内）で、ROS 2 環境を source して確認します。

```bash
source /opt/ros/humble/setup.bash

ros2 topic list                                          # トピックが出現しているか
ros2 topic echo /aiformula_control/gamepad/cmd_vel        # 実際に Twist が流れているか
ros2 topic hz /aiformula_control/gamepad/cmd_vel          # 10Hz 前後で来ているか

ros2 topic hz /aiformula_sensing/zed_node/left_image/undistorted/compressed   # 画像も15Hz前後で来ているか
ros2 run rqt_image_view rqt_image_view /aiformula_sensing/zed_node/left_image/undistorted/compressed  # 映像を目視確認

ros2 topic echo /aiformula_sensing/vectornav/imu    # IMUが流れているか
ros2 topic hz /aiformula_sensing/vectornav/imu      # レートを確認（下記の制約参照）

ros2 topic echo /aiformula_sensing/gyro_odometry_publisher/odom   # オドメトリが流れているか
ros2 topic hz /aiformula_sensing/gyro_odometry_publisher/odom     # レートを確認

ros2 topic echo /aiformula_sensing/vehicle_info     # 車輪速CANが流れているか（id: 1809）
ros2 topic hz /aiformula_sensing/vehicle_info       # レートを確認
```

ブラウザで W/A/S/D を押しながら `ros2 topic echo` を見て、`linear.x` / `angular.z` の値が
キー入力に応じて変化すれば、ROS 側まで正しく届いています。

`ros2 topic list` に出てこない場合は、rosbridge が本当に起動しているか、HUD の URL/topic
欄の入力ミス、ブラウザの開発者コンソールに WebSocket 接続エラーが出ていないかを確認して
ください。

## 操作方法

| キー | 動作 |
| --- | --- |
| W | 前進（押しっぱなしで加速） |
| S | 後退 / ブレーキ（前進中に押すと減速、停止後は後退） |
| A / D | 左右旋回（角速度を加減速） |
| R | 車体位置をリセット |
| マウスドラッグ / ホイール | 視点回転 / ズーム（OrbitControls） |

キーを離すと即停止せず、走行抵抗により**惰性で減速**します（下記の物理パラメータ参照）。

HUD 下部の「視点回転」「視点リセット」ボタンで、車体を360度好きな角度から確認できます。

- **視点回転**: 車体を中心に自動でカメラを一周させるトグルボタン（再度押すと停止）
- **視点リセット**: カメラを標準の追従視点（車体後方・斜め上）に戻す

視点（距離・角度）はマウスドラッグ/ホイールでいつでも自由に変更できます。車体が走行
していてもカメラは常にその角度・距離を保ったまま追従するので、手動で回した視点が
毎フレーム引き戻されることはありません。

画面右上には、車体に搭載された ZED カメラ位置（`config/zedx/extrinsic/extrinsic.yaml`
の取り付けオフセット）から前方を見た**機体カメラ視点**をピクチャーインピクチャーで
常時表示します。

## 機体カメラ映像の配信（compressed image）

rosbridge 接続中は、右上 PiP と同じ機体カメラ視点を `sensor_msgs/msg/CompressedImage`
として配信し続けます。

- **topic**: `/aiformula_sensing/zed_node/left_image/undistorted/compressed`
  （実車の `topic_list.yaml` にある `sensing.zedx.left_image.undistorted` に、
  `image_transport` の標準的な `/compressed` サフィックスを付けたもの）
- **frame_id**: `zed_left_camera_optical_frame`（`zed_macro.xacro` の命名に準拠）
- **format**: `jpeg`（画質 0.7、解像度 640x360 固定、[`js/simulator.js`](js/simulator.js)
  の `CAPTURE_WIDTH` / `CAPTURE_HEIGHT` / `CAPTURE_JPEG_QUALITY` で調整可能）
- **配信レート**: 15Hz（`IMAGE_PUBLISH_HZ`）

実装は、PiP 表示用とは別のオフスクリーン `WebGLRenderer` でオンボードカメラを毎回
640x360 に描画し、`canvas.toDataURL('image/jpeg', ...)` で得た base64 文字列を
そのまま `CompressedImage.data` に詰めて publish しています（rosbridge はバイト配列
フィールドに base64 文字列が来ると自動でデコードするため、これで正しい
`uint8[]` として届きます）。

## IMU（VectorNav）の配信・可視化

搭載されている VectorNav IMU を模した `sensor_msgs/msg/Imu` を rosbridge 接続中に配信します。

- **topic**: `/aiformula_sensing/vectornav/imu`（`topic_list.yaml` の
  `sensing.vectornav.imu` と同じ）
- **frame_id**: `vectornav`（実車の `sensing/vectornav/vectornav/config/vectornav.yaml`
  と同じ命名）。ただし base_link に対する実際の取り付けオフセットが不明なため、
  シミュレータ上では **base_link と同一位置**として扱い、車体の姿勢・速度から
  直接センサ値を計算しています。
- **配信レート**: 100Hz（[`js/simulator.js`](js/simulator.js) の `IMU_PUBLISH_HZ`）。
  描画ループ（`requestAnimationFrame`、通常60fps程度）とは切り離した専用の
  `setInterval` で配信しているため、描画フレームレートに左右されず実際に100Hzで
  publish されます。ただし車体の物理状態（速度・角速度など）自体は描画フレームと
  同じ ~60Hz でしか更新されないため、同じ描画フレーム内で送られる複数のIMU
  メッセージは同一の値を持つことがあります（伝送レートのみ100Hz）。
- **中身**:
  - `orientation`: yaw のみのクォータニオン（REP 103 規約に準拠し、Yaw は常に `-180° 〜 +180°` / `[-π, π]` に正規化。ロール・ピッチは 0）
  - `angular_velocity.z`: 旋回角速度（HUD の「角速度」と同じ値）
  - `linear_acceleration`: `x`=前後加速度（`vehicle_physics.js` の `linearAccel`）、
    `y`=旋回による遠心成分（`v * omega` の近似）、`z`=重力反力として固定 `+9.81`
  - 共分散はすべて 0（ノイズなしの合成データのため、実センサのノイズはモデル化していません）

**可視化**: HUD 左下に IMU の数値パネル（加速度 X/Y/Z・角速度 Z・Yaw）を常時表示します。

## オドメトリ（gyro_odometry_publisher）の配信・可視化

上記の IMU を使い、実車の `sensing/odometry_publisher`（`gyro_odometry_publisher`）
と同じ融合ロジックで `nav_msgs/msg/Odometry` を配信します。

- **topic**: `/aiformula_sensing/gyro_odometry_publisher/odom`
  （`topic_list.yaml` の `sensing.odometry.gyro` と同じ）
- **frame_id / child_frame_id**: `odom` / `base_footprint`
  （実際の起動パラメータ `odom_frame_id`/`vehicle_frame_id` と同じ）
- **配信レート**: 100Hz（`gyro_odometry_publisher.yaml` の
  `publish_timer_loop_duration: 10ms` に合わせた値。IMU と同様、描画ループとは
  切り離した専用の `setInterval` で配信）
- **アルゴリズム**: 実装（`odometry_publisher.cpp` の `updatePosition`/
  `createOdometryMsg`）をそのまま踏襲し、IMU由来の yaw・yaw rate と、車体の
  前後速度から `vx=v*cos(yaw)`, `vy=v*sin(yaw)` を計算して積分しています。
  - `pose.pose.position`: 積分した x, y（z=0）
  - `pose.pose.orientation`: yaw のみのクォータニオン
  - `twist.twist.linear`: `(vx, vy, 0)`（実装に合わせて **odom座標系** 表現。
    通常の ROS 規約が期待するボディ座標系ではない点に注意 — 実車のコードの
    挙動をそのまま再現しています）
  - `twist.twist.angular.z`: IMU の角速度
  - 共分散はすべて 0

  > **仕様**: `gyro_odometry_publisher` は VectorNav の IMU (`/aiformula_sensing/vectornav/imu`)
  > および車輪速 CAN (`/aiformula_sensing/vehicle_info`) を購読します。
  > 本シミュレータでもこれらと同じトピック名・データ形式（REP 103 規約の姿勢・ID 1809のCANフレーム）で
  > 配信しているため、シミュレータからのセンサデータで実機の ROS 2 ノードをそのまま動作させることができます。

**可視化**: HUD に位置(X/Y)・Yaw・速度(X/Y)・角速度のパネルを表示するほか、
3D シーン上に**水色の軌跡**としてオドメトリの走行経路を描画します（直近300点、
0.15秒間隔でサンプリング）。R キーでリセットすると軌跡もクリアされます。

## 車輪速CANの配信（実測値 vs 理論値）

`odometry_publisher/include/odometry_publisher/wheel.hpp` がデコードする形式に合わせて、車輪速を `can_msgs/msg/Frame` として配信します。

- **実測値 (Feedback / Measured RPM)**:
  - **topic**: `/aiformula_sensing/vehicle_info`（`topic_list.yaml` の `sensing.input_can_data`）
  - **CAN ID**: `1809` (`0x711`、`odometry_publisher` の `RPM_ID`)
  - **内容**: 70kg 車体の質量・慣性・走行抵抗を受けた**現在の実際の車輪回転数**。
  - **データ (8バイト)**: `data[0..3]` = 右輪RPM、`data[4..7]` = 左輪RPM（符号付き32bit・リトルエンディアン）
- **理論値 / 指令値 (Target / Commanded RPM)**:
  - **CAN ID**: `0x210` (`528`、`motor_controller.py` がモーターアンプへ送る指令)
  - **内容**: 入力キー（WASD）や自律走行 `cmd_vel` が要求する**目標車輪回転数**。キーを押した瞬間に目標値へ切り替わり、実測値が慣性に従って徐々に追従します。
- **RPM ⇔ 速度の変換**: `wheel.hpp` 側の decode 式
  `speed = rpm * (1/60) * (diameter * π)` に厳密に合わせ、逆算で
  `rpm = speed / (diameter * π) * 60` を使用（`diameter = 0.254m`）。
- **配信レート**: 100Hz（実車のCAN計測周期 ~10ms に合わせた値）

これを購読する `gyro_odometry_publisher`/`wheel_odometry_publisher` は、
本シミュレータが配信するオドメトリ(`/aiformula_sensing/gyro_odometry_publisher/odom`)
とは独立した別トピック(`sub_can`)からの入力であり、両者は連動していません
（本シミュレータのオドメトリは IMU + 車体速度から直接計算しています）。

## oit_navigation: 白線検出 → 周回マップ作成 → QP レーシングライン走行

`src/oit_navigation` が実車で行う走行方式を、そのままブラウザ内で動かせます
（アルゴリズム本体 [`js/lane_navigator.js`](js/lane_navigator.js) は
`src/oit_navigation/oit_navigation/lane_nav/` の JS 移植で、同じ入力で Python 版と結果が一致することを確認済み）。
シミュレータは **システム構成・技術の組み合わせがうまく動くかの検証** 用です（車両特性は今後追加予定）。

1. **白線検出**: カメラ画像から白線を検出し、地面 (base_link) に投影して **左境界 / 中央線 / 右境界** に割り当てる
   （線の並び順ではなく横位置で追跡。見えない線は道幅から補完 = 破線表示）
2. **1周目**: 中央白線の上を走りながら、前方 2m の左右境界点を odom 座標に記録
   （直線は 3m 間隔、カーブは 0.6m 間隔まで詰める）。スタート地点に戻ったら 1 周完了
3. **2周目以降**: 各断面の道幅の中にウェイポイントを置き、QP（曲率最小）で
   アウト・イン・アウトのラインを作って追従

### 使い方（「自動運転」タブ）

- **検出器**
  - **YOLOP**（既定）: 実車と同じ `models/honda_shihou_finetuned_best.pth`（[`js/lane_model_detector.js`](js/lane_model_detector.js)、
    `models/honda_shihou_finetuned.onnx`）の白線マスクを、`extractMaskLines()` で線ごとの点列にする
  - **UFLD**: `models/ufld.onnx`（[`js/ufld_lane_detector.js`](js/ufld_lane_detector.js)）。245MB で git 管理外のため、
    `models/ufld_honda_finetuned_best.pth` を置いて `ros2 run oit_navigation export_ufld_onnx` で生成する
  - **理想検出**: コースの実際の 3 本線（[`js/course_lines.js`](js/course_lines.js)、
    [`tools/extract_course_lines.py`](tools/extract_course_lines.py) でコース画像から抽出）を
    ノイズ・欠落つきで観測する。認識精度と走行方式を切り分けて検証するためのモード
  - **ROS2連携**: 下記「ROS 2連携モード」
- **スタート位置へ**: 車両を外周の中央白線の上 `(0, -1.6, 0)` に置き、記録を 1 周目からやり直す
- **1周目終了**: 周回検出を待たずに記録を確定し QP ラインを作る（実機の `/lane_navigator/finish_mapping` と同じ）
- **リセット**: 記録を消して 1 周目から
- **自動運転: ON/OFF**: twist_mux の `mpc` 入力の有効/無効

右のパネル: 上 = 白線検知（白点 = 検出点、水色/黄/桃 = 左/中央/右、破線 = 補完）、
下 = 周回マップ（水色/桃の点 = 記録した左右境界、橙の○ = QP ウェイポイント、青 = 自己位置の軌跡）。

自己位置は実車の `odom_imu_localizer` と同じく車輪速 + ヨーレートの積算（真値ではない）です。
コースの中央線 <-> 境界線は約 3.1m（`SIM_LANE_WIDTH`）、速度上限はシミュレータの WASD と同じ 1.5 m/s。

### 自動検証用の早送り（ブラウザのコンソール）

タブが裏に回ると描画ループが止まるため、検証は描画に依存しない早送りで行えます。

```js
window.__sim.setDetectorMode('ideal');          // 'yolop' | 'ufld' | 'ideal'
document.getElementById('start-pose-btn').click();
document.getElementById('autonomous-btn').click();
await window.__sim.fastForward(300);             // シミュレーション時間で 300 秒
window.__sim.laneNavigator.status();             // {state, lap, samples, ...}
```

### twist_mux（WASD と自動運転の調停）

「自動運転」ON中も、WASD を押せば**即座に手動操作が優先**され、離すと自動運転に
戻ります。これは実車の `twist_mux`（[`control/twist_mux`](../control/twist_mux)、
優先度は [`launchers/sample_launchers/config/twist_mux.yaml`](../launchers/sample_launchers/config/twist_mux.yaml)）
と同じ「優先度＋タイムアウト」方式をそのまま [`js/twist_mux.js`](js/twist_mux.js) に
移植したもので、単純な二択トグルではありません。

| ソース | 実車トピック | 優先度 | タイムアウト |
| :--- | :--- | ---: | ---: |
| gamepad（WASD） | `/aiformula_control/gamepad/cmd_vel` | 150 | 0.3秒 |
| mpc（自動運転） | `/aiformula_control/extremum_seeking_mpc/cmd_vel` | 50 | 0.3秒 |

WASD のいずれかのキーを押している間だけ gamepad ソースが「新鮮」とみなされ、
優先度の高い gamepad が mpc を上書きします。キーを離してから 0.3 秒経つと
gamepad がタイムアウトし、自動運転（有効になっていれば）に制御が戻ります。
実車の `handle_controller`（優先度250、物理ハンドル）と
`handle_controller_coasting`（優先度1）はこのシミュレータに対応する入力が
ないため移植していません。

調停結果（どちらのソースが新鮮か・現在採用中のソース）は HUD の
「twist_mux (制御調停)」パネルに表示されます。調停後の実際の指令値は、実車の
`cmd_vel_out` 相当の `/aiformula_control/twist_mux/cmd_vel`
（`topic_list.yaml` の `control.speed_command.multiplexed`）としても配信されます。

### トピック名の統一

ブラウザ内モード (YOLOP/UFLD/理想検出) でも、実車の `lane_detector` / `lane_navigator` と同じトピック名で配信します（rosbridge 接続時）。

| トピック | 型 | 内容 |
| :--- | :--- | :--- |
| `/aiformula_visualization/lane_detector/annotated_image` | `Image` (rgb8) | 白線検知パネルの画像 |
| `/aiformula_perception/lane_line_publisher/lane_lines/{left,center,right}` | `nav_msgs/Path` (base_link) | 左/中央/右の白線 |
| `/aiformula_visualization/lane_navigator/{left,right}_boundary` | `nav_msgs/Path` (odom) | 記録した左右境界 |
| `/aiformula_visualization/target_trajectory` | `nav_msgs/Path` (odom) | QP レーシングライン |
| `/aiformula_control/extremum_seeking_mpc/cmd_vel` | `Twist` | 自動運転指令（twist_mux の mpc） |
| `/aiformula_control/lane_tracker/status` | `String` (JSON) | 走行状態 |

### ROS 2連携モード（実機と同じ ROS 2 ノードで走らせる）

「ROS2連携」を選ぶと、ブラウザは検出・計算をせず、ROS 2 側の
`lane_detector` → `lane_navigator`（+ `odom_imu_localizer`）の出力で走ります。

```bash
make rosbridge                 # 1. rosbridge_server
# 2. ブラウザで http://localhost:8000/web_simulator/ を開いて「接続」
make sim-nav                   # 3. simulator_test.launch.py (BACKEND=yolop|ufld)
# 4. 「自動運転」タブで「ROS2連携」→「スタート位置へ」→「自動運転: ON」
```

| 方向 | トピック | 型 | 内容 |
| :--- | :--- | :--- | :--- |
| Sim→ROS | `/aiformula_sensing/zed_node/left_image/undistorted/compressed` | `CompressedImage` | 機体カメラ映像 → `lane_detector` |
| Sim→ROS | `/aiformula_sensing/vehicle_info` | `can_msgs/Frame` | 車輪速 CAN → `odom_imu_localizer` |
| Sim→ROS | `/aiformula_sensing/vectornav/imu` | `Imu` | ヨーレート → `odom_imu_localizer` |
| ROS→Sim | `/aiformula_control/extremum_seeking_mpc/cmd_vel` | `Twist` | `lane_navigator` の指令 → `twistMux.update('mpc', ...)` |
| ROS→Sim | `/aiformula_visualization/lane_detector/annotated_image` | `Image` (bgr8) | 白線検知パネル |
| ROS→Sim | `/aiformula_control/lane_tracker/status` | `String` (JSON) | HUD の走行状態 |
| ROS→Sim | `/aiformula_visualization/lane_navigator/{left,right}_boundary`, `/aiformula_visualization/target_trajectory` | `Path` | 周回マップパネル |
| Sim→ROS | サービス `/lane_navigator/finish_mapping`, `/lane_navigator/reset` | `std_srvs/Trigger` | 「1周目終了」「リセット」ボタン |

シミュレータのカメラは光軸中心の理想ピンホールのため、`simulator_test.launch.py` は
`camera_cx/cy` を画像中心、`lane_width` を 3.1m、速度上限をシミュレータに合わせて起動します。

### 実装上の注意

- ONNX モデル（YOLOP / UFLD）は onnxruntime-web で推論します。WebGPU が使えなければ CPU の WASM にフォールバックします。
  スレッド版 WASM は SharedArrayBuffer（= COOP/COEP ヘッダー）が必要なため、必ず [`serve.py`](serve.py) で配信してください。
- YOLOP の前処理はブラウザ側では `crop_bottom`（上部を切り落として 640x640 に引き伸ばし）、
  チャンネル順は学習時と同じ BGR のまま ImageNet mean/std で正規化します。

## 車体モデル・物理パラメータ

`vehicles/sample_vehicle/xacro/ai_car1.xacro` に準拠した差動2輪駆動（後方キャスター）
構成です。パラメータは [`js/vehicle_physics.js`](js/vehicle_physics.js) にまとめています。

- 車体質量: **70 kg**（ユーザー指定。加減速・惰性の計算に実際に使用）
- 駆動輪半径: 0.12 m（xacro `WHEEL_RADIUS`）
- トレッド幅: 0.6 m（`config/wheel.yaml` の `tread`）
- 最高速度: 1.5 m/s、最高後退速度: 0.75 m/s
- 最大旋回角速度: 1.2 rad/s

加減速・ブレーキ・惰性走行時の抵抗はすべて「力 [N] ÷ 質量 [kg] = 加速度」の形で
計算しており、質量を変えると挙動が変わります。

## ファイル構成

```
web_simulator/
├── index.html              HUD と canvas、importmap
├── js/
│   ├── simulator.js         シーン構築・メッシュ読込・入力・rosbridge通信・描画ループ
│   ├── vehicle_physics.js   差動2輪駆動の物理モデル（質量70kg）
│   ├── course.js            コースレイアウトの地面テクスチャ生成
│   ├── lane_model_detector.js      YOLOP (ONNX) による白線マスク検出（crop_bottom前処理）
│   ├── ufld_lane_detector.js       UFLD (ONNX) による白線点列検出
│   ├── ideal_lane_detector.js      理想検出（コースの実際の白線 + ノイズ）
│   ├── course_lines.js             外周コースの 3 本線（tools/extract_course_lines.py で生成）
│   ├── lane_navigator.js           oit_navigation lane_nav の JS 移植（線追跡・境界記録・QP・追従）
│   └── twist_mux.js         WASD/自動運転の優先度＋タイムアウト調停
├── tools/extract_course_lines.py   コース画像から 3 本線を抽出
├── models/                  honda_shihou_finetuned.onnx（export_onnx_web.py）/ ufld.onnx（export_ufld_onnx.py, git 管理外）
└── vendor/                  three.js / roslib.js / onnxruntime-web のローカル同梱コピー
```

## コースレイアウト

ユーザー提供のコースレイアウト画像（[`png/shihou_cource_unity.png`](png/shihou_cource_unity.png)、
Unity上のコース設計ツールのスクリーンショット）を、[`js/course.js`](js/course.js) で
そのまま地面テクスチャとして読み込み、車体の初期位置を中心に敷いています。

- 実寸法（外形約98.9m×91.4m、道幅9.0mなど）は現時点では未反映です（指示によりいったん保留）。
  地面の大きさは画像のアスペクト比（1024:819）だけを使い、幅100m相当で仮に配置しています
- 画像をそのまま貼っているだけの **見た目のみ** の実装です。道路外への進入判定や
  ゲート通過判定などのロジックは実装していません（車体はこれまで通り
  どこでも自由に走行できます）。周回の判定は oit_navigation の自己位置ベースで行います
- 実寸法を反映する場合は `js/course.js` の `COURSE_WIDTH_M`（現在100m固定）を
  実際のコース幅に合わせて変更してください
- 車体の初期位置は odom 原点 `(0,0,0)` のままです。コース画像は
  [`js/simulator.js`](js/simulator.js) の `COURSE_POSE` で現在
  `(x: 13.22, y: 35.41, yaw: +90°)` に配置しています

## 既知の制約

- 読み込むメッシュは車体 (`AIF_body.dae`) と車輪 (`tire.dae`) のみです。
  ZED カメラの `zedx.stl`（約13MB）は読み込み時間短縮のため対象外にしています。
- 衝突判定・地形は未実装です（コースの路面はテクスチャのみで、道路外進入の制限はありません）。
