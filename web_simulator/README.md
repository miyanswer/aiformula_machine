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

Web シミュレータの HUD の「詳細」タブ →「rosbridge」で以下を設定し、「接続」ボタンを押します。

- **rosbridge URL**: 既定値 `ws://localhost:9090`（コンテナ越しにアクセスする場合は
  ホスト名/IPを書き換えてください）
- **cmd_vel topic**: 既定値 `/aiformula_control/gamepad/cmd_vel`
  （`launchers/sample_launchers/launch/twist_mux.launch.py` が実際に購読している
  gamepad 入力トピック。twist_mux の priority は 150。
  `control/twist_mux/config/twist_mux_topics.yaml` の `keyboard`（`key_vel`）経由で
  twist_mux を単体起動している場合は、ここを `key_vel` に変更してください）

- **実機操縦（cmd_vel のみ送信）**: 実機（Jetson）の rosbridge に繋いで WASD で操縦する
  ときはオン（URL が localhost 以外なら自動でオン）。キーを押している間だけ cmd_vel を送り、
  離すと速度 0 を送って止めます。「自動運転: ON」の間だけ、シミュレータの自律走行指令を
  `/aiformula_control/extremum_seeking_mpc/cmd_vel`（実機 twist_mux の mpc 入力）にも送り、
  実機をシミュレータの車と同じ指令で走らせます（OFF・タブ非表示で速度 0 を送って停止）。
  シミュレータのセンサ・画像・twist_mux 出力は一切送りません（オフのまま実機に繋ぐと、
  それらが実機の同名トピックに流れ込みます）。
  以下の説明はオフ（ローカルの ROS 2 スタックとシミュレータを連携させる通常モード）のものです。

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

HUD の「走行」タブの「視点回転」「視点リセット」ボタンで、車体を360度好きな角度から確認できます。

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

**可視化**: HUD の「詳細」タブに IMU の数値（加速度 前後/左右/上下・角速度・Yaw）を表示します。

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

**可視化**: HUD 上部に位置(x/y)・向き（Yaw）を常時表示し、「詳細」タブに速度(X/Y)・角速度を表示するほか、
3D シーン上に**水色の軌跡**としてオドメトリの走行経路を描画します（直近300点、
0.15秒間隔でサンプリング）。R キーでリセットすると軌跡もクリアされます。

## 車輪速CANの配信（実測値 vs 理論値）

`odometry_publisher/include/odometry_publisher/wheel.hpp` がデコードする形式に合わせて、車輪速を `can_msgs/msg/Frame` として配信します。

- **実測値 (Feedback / Measured RPM)**:
  - **topic**: `/aiformula_sensing/vehicle_info`（`topic_list.yaml` の `sensing.input_can_data`）
  - **CAN ID**: `1809` (`0x711`、`odometry_publisher` の `RPM_ID`)
  - **内容**: 71.6kg 車体の質量・慣性・走行抵抗を受けた**現在の実際の車輪回転数**。
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

### 使い方（「自動運転」タブ・「走行」タブ）

- **検出器**
  - **YOLOP**（既定）: 実車と同じ `models/honda_shihou_finetuned_best.pth`（[`js/lane_model_detector.js`](js/lane_model_detector.js)、
    `models/honda_shihou_finetuned.onnx`）の白線マスクを、`extractMaskLines()` で線ごとの点列にする
  - **UFLD**: `models/ufld.onnx`（[`js/ufld_lane_detector.js`](js/ufld_lane_detector.js)）。245MB で git 管理外のため、
    `models/ufld_honda_finetuned_best.pth` を置いて `ros2 run oit_navigation export_ufld_onnx` で生成する
  - **コーン検知**: `models/cone.onnx`（[`js/cone_detector.js`](js/cone_detector.js)）。git管理外のため、
    `models/cone.pt` を置いて `ros2 run oit_navigation export_cone_onnx` で生成する。
  - **信号機検知**: `models/traffic_light.onnx`（[`js/traffic_light_detector.js`](js/traffic_light_detector.js)）。git管理外のため、
    `python3 src/oit_navigation/oit_navigation/export_cone_onnx.py --weights models/traffic_light.pt --output web_simulator/models/traffic_light.onnx`
    で生成する（下記「赤信号停止」）。
  - **理想検出**: コースの実際の 3 本線（[`js/course.js`](js/course.js) が `course.glb` の
    メッシュから取り出した外側境界・中央線・内側境界の点列）をノイズ・欠落つきで観測する。
    認識精度と走行方式を切り分けて検証するためのモード
  - **ROS2連携**: 下記「ROS 2連携モード」
- **スタート位置へ**（「走行」タブ）: 車両を中央白線のスタート位置 `(0, 0, 0)` に戻し、記録を 1 周目からやり直す
- **1周目終了**: 周回検出を待たずに記録を確定し QP ラインを作る（実機の `/lane_navigator/finish_mapping` と同じ）
- **記録リセット**: 記録を消して 1 周目から
- **自動運転: ON/OFF**: twist_mux の `mpc` 入力の有効/無効

右のパネル: 上 = 白線検知（白点 = 検出点、水色/黄/桃 = 左/中央/右、破線 = 補完）、
下 = 周回マップ（水色/桃の点 = 記録した左右境界、橙の○ = QP ウェイポイント、青 = 自己位置の軌跡）。

自己位置は実車の `odom_imu_localizer` と同じく車輪速 + ヨーレートの積算（真値ではない）です。
コースの中央線 <-> 境界線は 3.5m（`course.glb` から実測、`SIM_LANE_WIDTH`）、速度上限はシミュレータの WASD と同じ 1.5 m/s。

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

調停結果は、HUD ヘッダのバッジ（「手動」/「自動運転」/「待機」）と、「詳細」タブの
「制御の調停 (twist_mux)」（どちらのソースが新鮮か・現在採用中のソース）に表示されます。調停後の実際の指令値は、実車の
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

## 6レーン動的選択走行（地図なし・オドメトリなし）

「自動運転」タブの **走行方式** で「周回マップ+QP」（既存）と「6レーン (地図なし)」を切り替えます。
6レーン方式は地図を作らず、自己位置（オドメトリ）も使わず、その瞬間のカメラ白線だけで走ります。

- 左白線〜中央線、中央線〜右白線をそれぞれ3等分した仮想レーン **L1〜L6**（左端が L1、右端が L6）を毎フレーム作る
- 白線の点群から前方 3m / 6m / 9.5m の曲率を求め、速度と合わせて小さなニューラルネット（MLP）が
  行くべきレーンを確率で出す（左回りなので、直線=L6 → カーブ中=イン側 L1〜L2 → 出口=L6 のアウト・イン・アウト）
- コーンで塞がれたレーンは除外、目標レーンへは進入角を制限した Pure Pursuit で移る。速度は車輪速（CAN 相当）だけ使う
- 右上の俯瞰パネルが「6レーン 白線点群 (俯瞰)」に変わり、左/中央/右白線の点群・6レーン（現在=緑、目標=橙、
  コーンで閉塞=赤）・NN の確率バー・予定軌跡を表示
- **右下**に日本語で思考結果（現在レーン → 目標レーン、局面、判断理由、NN の各レーン確率）を表示

白線検出は「理想検出」「YOLOP」「UFLD」のどれでも動きます。「ROS2連携」を選ぶと ROS 2 の
`six_lane_planner` ノード（`ros2 launch oit_navigation six_lane.launch.py simulator:=true`）の判断を右下に表示します。
アルゴリズムと ROS 2 ノードの詳細は [`src/oit_navigation/oit_navigation/6lane/README.md`](../src/oit_navigation/oit_navigation/6lane/README.md)。
NN の重み `six_lane_policy.json` は実機ノードと共用で、シミュレータは `../src/oit_navigation/oit_navigation/6lane/` から読み込みます。

コンソールからの検証: `__sim.setNavMethod('sixlane')` → `await __sim.fastForward(200)`（返り値は status JSON）。

確認結果（早送り）: YOLOP 2 周は逸脱 0 回・接触 0 回（最小余裕 0.19m）。理想検出は 22 周中 1 周だけ白線に
0.12m 触れた（残り 21 周は逸脱なし）。各カーブで L6 → L1〜L2 → L6 のアウト・イン・アウト。
直線のコーン 1 本（L6 上）は L5 に移って 1.2m 以上離して通過。
白線の追跡（LineTracker）は「車両は中央線の上」と仮定しない: 見失っても横位置を保ち、3本揃い・二重線を根拠に
取り違えを付け直す（YOLOP で右端に何も伝えずに置いても 0.2 秒で正しい横位置に戻る）。詳細は 6lane/README.md。

## 赤信号停止（両方の走行方式）

MyLaps ゲートのパネルが信号です。**実機と同じ `traffic_light.pt`**（ONNX に変換）でオンボードカメラ画像から
赤/青信号を検出し、バウンディングボックスの縦の画面占有率から距離を逆算します（実機の `traffic_light_distance_node` と同じ式）。
赤を連続 2 フレーム見たら、**信号機の 7.0m 手前（許容 5〜10m）** で止まるよう `v <= sqrt(2·0.6·(d − 7.0))` で減速し、
検出の合間は車輪速で距離を補間します。停止中に青を 2 フレーム見るか、赤が 3 秒見えなくなったら発進します。
赤を初めて見た距離が 5m より近い（直前で赤に変わった）ときは、止まらずに通過します。
走行方式（周回マップ+QP / 6レーン）の最終指令に掛かるので、どちらでも同じように止まります。

- 「自動運転」タブの **コースの信号** で「赤⇔緑 (10秒)」（既定: 10 秒ごとに赤と緑を交互に切替）/「赤固定」を選ぶ
- 「信号機」欄にコースの信号の色・切替までの秒数、停止制御の状態（通常/減速中/停止中/発進中）と検出距離を表示。
  6レーン方式では右下の判断パネルにも「信号: 赤信号で停止中 (信号機まで 6.7m)」のように出る
- 距離の焦点距離は **900px @1080 に実測校正**しています（シミュレータのカメラの幾何的な値は 763px）。YOLO のボックスは
  小さい物体ほど実物より数 px 大きく出るため、幾何的な値のままだと距離を 1〜2m 短く見積もります。3〜18m に置いた
  赤/緑パネル 120 枚で合わせ、停止帯 4〜8m で誤差平均 ±0.1m。12m より遠いとボックスが頭打ちになり短めに出ます
  （= 早めに減速し始めるだけ）
- 検証（`__sim.fastForward`）: 理想検出で 6レーン×4 開始位置 + QP の停止位置は信号機まで **7.0〜7.7m**、
  YOLOP では 7.1〜8.0m。赤⇔緑では減速中に緑になれば止まらずに通過し、停止後は緑で発進してゲートを通過
- 実装: [`js/traffic_light_stop.js`](js/traffic_light_stop.js) ≡ `src/oit_navigation/oit_navigation/utils/traffic_light_stop.py`
  （同じ入力列で状態遷移・速度とも一致を確認済み）
- ROS2連携モードでは実機ノード（`traffic_light_distance_node` + `lane_navigator` / `six_lane_planner`）が止め、
  シミュレータは `/aiformula_control/traffic_light_stop/status` を表示するだけ。ブラウザ内の検出モードでは
  シミュレータ自身が `/aiformula_perception/traffic_light/{red,green}_distance` と同じ status を出す
- コーン回避 (js/cone_avoidance.js) は実機の `lane_nav/cone_avoidance.py` と同一。2 周目のライン押し出しは
  ラインを細かくしてからコースに収まる側へ 1.3m 離す (以前は疎なウェイポイントを 1 点押すだけで, 間のスプラインが
  コーンの脇を通っていた)。1 周目のコーン記憶はコースマップと同じ補正で地図に載せる (以前は方位ドリフト補正を常に掛け,
  ループ閉じ込みを掛けていなかったため, 境界地図とずれることがあった)
- コーン検知の結果もブラウザ内の検出モードでは実機の `cone_detector` と同じ `/aiformula_perception/cone_detector/cones`
  (PoseArray, base_link) に出す。RViz 用のマーカー・判断パネル画像は実機ノードだけが出すので、シミュレータで RViz 確認するときは
  ROS2連携モード（`simulator_test.launch.py` / `six_lane.launch.py simulator:=true`）を使う

## 車体モデル・物理パラメータ

`vehicles/sample_vehicle/xacro/ai_car1.xacro` に準拠した差動2輪駆動（後方キャスター）
構成です。パラメータは [`js/vehicle_physics.js`](js/vehicle_physics.js) にまとめています。

- 車輪荷重: 右 **25.0 kg**、左 **23.4 kg**、後 **23.2 kg**（合計 **71.6 kg**。加減速・惰性の計算に実際に使用）
- ホイールベース: **0.815 m**（前輪軸から後輪軸まで）
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
│   ├── vehicle_physics.js   差動2輪駆動の物理モデル（質量71.6kg）
│   ├── course.js            course.glb の読込と、コース寸法・スタート位置・白線点列・中央線パスの実測
│   ├── course_props.js      MyLaps ゲート（models/MyLaps.obj）の読込・配置・当たり判定用の円
│   ├── collision.js         2D 当たり判定ヘルパー（円と障害物、押し戻し、コース逸脱の判定）
│   ├── cone_props.js               コーンの3Dモデル(models/cone.glb)読み込み・当たり判定
│   ├── cone_editor.js              コーンのクリック配置・ドラッグ移動・削除 (localStorage永続化)
│   ├── cone_detector.js            cone.onnx によるコーン検知 (バウンディングボックス -> 地面座標)
│   ├── cone_avoidance.js           コーン回避 (反応的ナッジ・1周目記憶・2周目レーシングライン回避・ランドマーク補正)
│   ├── traffic_light_detector.js   traffic_light.onnx による信号機検知 (赤/青 + 画面占有率から距離)
│   ├── traffic_light_stop.js       赤信号停止 (oit_navigation/utils/traffic_light_stop.py の JS 移植)
│   ├── lane_model_detector.js      YOLOP (ONNX) による白線マスク検出（crop_bottom前処理）
│   ├── ufld_lane_detector.js       UFLD (ONNX) による白線点列検出
│   ├── ideal_lane_detector.js      理想検出（コースの実際の白線 + ノイズ）
│   ├── lane_navigator.js           oit_navigation lane_nav の JS 移植（線追跡・境界記録・QP・追従）
│   ├── six_lane_planner.js         6レーン動的選択走行（oit_navigation/6lane/six_lane_core.py の JS 移植）
│   ├── hud_ui.js                   HUD のタブ切替・折りたたみ・ドラッグ移動
│   └── twist_mux.js         WASD/自動運転の優先度＋タイムアウト調停
├── models/                  course.glb（コース）/ MyLaps.obj・.mtl（ゲート）/ cone.glb（コーン）/
│                            honda_shihou_finetuned.onnx / ufld.onnx / cone.onnx / traffic_light.onnx（git 管理外）
└── vendor/                  three.js（GLTFLoader を含む）/ roslib.js / onnxruntime-web のローカル同梱コピー
```

## コース

コースは [`models/course.glb`](models/course.glb) をそのまま使います（Blender からの glTF 書き出し）。
モデルは ROS 規約（地面 = xy 平面、高さ = z、単位 m）で作られていて、ルートノードの回転が
`rosRoot` の回転を打ち消すため、`rosRoot` の子に置くだけでスケールや向きの補正は不要です。
アスファルトの路面（`Course_Base`）の上面が `z = 0` の走行面で、白線は路面の上に 5 mm の厚みで乗っています。

`course.glb` を差し替えると、以下はすべてモデルから読み直されます（[`js/course.js`](js/course.js)）。

| 項目 | 現在の `course.glb` での値 |
| --- | --- |
| 白線の幅 | 0.15 m |
| 車線幅（中央線 ↔ 境界線） | 3.5 m |
| 中央線 1 周 | 約 258.8 m |
| スタート位置 | コース座標で (-41.457, 15.109)、向き -91.42°（左辺・南向き）。白線リボンの中心に載り（誤差 0.13 mm）、向きも線と 0.03° 以内。ここが odom の原点 |

- 材質は元の色のまま**ライティングなし**（`MeshBasicMaterial`）に置き換えています。機体カメラの画像は
  YOLOP/UFLD の入力なので、太陽の角度で見え方が変わらないようにするためです。
- スタート位置は [`js/course.js`](js/course.js) の `START_POSE`（コース座標）で**手で指定**しています。
  中央実線のリボンの頂点（線の両縁、±7.5 cm）を最小二乗で当てはめて求めた、線の中心と向きです。
  モデルを書き出し直してループの位置が変わったら、この値を当てはめ直してください。
- 走行方向は**左回り（反時計回り）**です。スタートは左辺で、南へ向かって走り出します。
- モデル内の `Cube` ノード（Blender 既定の立方体）は、あれば読み込み時に捨てます。

### 座標系: スタート位置が odom の (0, 0)

コースはスタート位置を **odom の原点 (0, 0)、向きを +x** にするよう平行移動・回転して置いています。
車体は `(0, 0, 0)` から始まるので、

- 車体の初期位置 ＝ 中央白線のスタート位置（線の上）
- オドメトリの `(0, 0)` ＝ そこ
- 「スタート位置へ」ボタン・R キーも同じ場所に戻す

になります。コース座標との対応はモデルの座標を回転・平行移動したもので、`course.js` が `root` に設定します。

### 当たり判定・コース逸脱

- **障害物（MyLaps ゲートの支柱とコーン）**: 物理的に阻止します。車体は base_link 上の
  2 円（半径 0.40 m、中心は x = +0.30 m と x = -0.50 m）で近似し、貫入した分だけ押し戻します。
  正対して当たった場合は前進速度を 0 にします。ゲート（信号機）は odom 座標の (0.65, 78.60) に置いています
  （中央線上、スタートから進行方向に約 150 m の西向きの直線。LED パネルとコーンが接近側を向く）。
  座標は [`js/course_props.js`](js/course_props.js) の `MYLAPS_POSITION` で、スタート位置を動かすと
  一緒に動きます。支柱の間隔は 0.46 m で車幅より狭く、
  通り抜けられないのは意図的です（今後コーン検知 YOLO を入れたときの回避対象）。
- **コース逸脱**: 走行は止めず、HUD に「逸脱中」表示と累計回数を出すだけです。中央線パスからの横ずれが
  3.175 m（`車線幅 + 線幅/2 - 車体半幅`）を超えた時点で 1 回と数え、3.0 m 以内に戻るまで次を数えません。
  ROS トピックは発行しません（実車に対応するノードが存在しないため）。

### 自動運転での確認結果

理想検出モードで新コースを自動運転させた結果です（ゲートを無効にした場合）。

- マッピング周（1 周目）: 258.8 m の外周を中央線から ±0.06 m 以内で走破、逸脱 0 回
- 続くレーシングライン走行（1.5 m/s）: 中央線から最大 約 2.8 m 外れる（コーナーを短絡するため）が、逸脱 0 回
- **`js/lane_navigator.js` に変更は不要**でした

ゲートを有効にすると、マッピング周はゲートの手前 1.43 m（速度 0）で止まります（スタートから 210 m 地点）。一方
**レーシングラインはゲートを素通りします**（ゲート位置で中央線から約 2.6 m 外側を走り、最接近 2.61 m）。
ゲートは中央線を走る周回だけを止めます。

## コーン配置・検知・回避

詳細設計は [`docs/superpowers/specs/2026-09-22-cone-avoidance-design.md`](../docs/superpowers/specs/2026-09-22-cone-avoidance-design.md) を参照。ここでは要点のみ。

- **配置**: 「走行」タブの「コーン配置」ボタンを ON にすると、3D ビュー上のクリックでコーンを追加、
  既存コーンのクリックで削除、ドラッグで移動できます（[`js/cone_editor.js`](js/cone_editor.js)）。本数無制限、
  レーン内かどうかの制約もありません。状態は `localStorage`（キー `aiformula_cones_v1`）に保存され、
  リロード後も復元されます。各コーンは [`js/cone_props.js`](js/cone_props.js) が `models/cone.glb` を
  読み込んで配置し、半径 0.15 m の円として `collision.js` の当たり判定（既存の MyLaps ゲート支柱と同じ扱い）に加えます。
- **検知**: `models/cone.onnx`（Ultralytics YOLO、`src/oit_navigation/oit_navigation/export_cone_onnx.py` で
  `models/cone.pt` から書き出し、git 管理外）を [`js/cone_detector.js`](js/cone_detector.js) が
  `lane_model_detector.js` と同じ WebGPU→WASM 構成でブラウザ推論します。オンボードカメラ画像
  (`captureCanvas`) を毎パーセプションティック推論し、バウンディングボックス下辺中央を
  `lane_navigator.js` の `projectToGround()`（白線検知と同じカメラモデル）でそのまま地面に投影するので、
  信号機検知のような距離逆算式は不要です（有効範囲 `0.3m < x < 8.0m`, `|y| < 3.0m`）。
  `models/cone.onnx` が無い場合は HUD「コーン検知」欄にエラーを表示するだけで、白線追従・衝突判定など
  他の機能には影響しません。
- **回避（[`js/cone_avoidance.js`](js/cone_avoidance.js)）**: 反応的回避と、1 周目終了時の後処理の 2 段構えです。
  - **反応的ナッジ（毎フレーム、1 周目・2 周目共通）**: `navigator.step()` の出力 `{v, omega}` に、検出中の
    コーンから離れる旋回バイアスを重ねて `physics` に渡します（`reactiveAvoid()`）。近距離のコーンには減速も
    かけます。あくまで一時的な補正で、地図やレーシングラインそのものは書き換えません。
  - **1 周目の記憶と 2 周目の後処理**: 1 周目（MAPPING）走行中、検出したコーンを `ConeRecorder` が
    「検出時の走行距離 `s` ＋ 車体ローカルオフセット」で記録し、境界点と同じ「補正後の姿勢列から再投影する」
    方式（`correctedPoseSequence()`、`lane_navigator.js` からの再利用）で世界座標に確定します。
    1 周目が終わり 2 周目（RACING）に入った瞬間に 1 回だけ、この確定済みコーン座標で
    `applyRacelineDeflection()` がレーシングラインの点列自体をコーンから遠ざける後処理を行います
    （反応的ナッジと異なり、経路そのものを恒久的に書き換える）。さらに 2 周目以降は
    `coneLandmarkCorrection()` が、記憶済みコーンと今の検知位置のズレからオドメトリ推定（`localizer`）の
    ドリフトを補正します。
- **既知の制約**: MyLaps ゲートの支柱はコーンと同形状・同寸法の飾りのため、コーン検知器はゲート通過時に
  これも「コーン」として検知します。支柱間隔（0.46 m、「当たり判定・コース逸脱」節）は車体より狭く、
  複数本を同時に避ける経路計画は反応的ナッジ（1 本ずつの単純な旋回バイアス）の対象外なので、ゲートに
  正対したまま足止めされることがあります。単独で離れて置かれたコーンの回避・記憶は正常に動作します。

## スリップ誤差モデル

`js/vehicle_physics.js` の車輪速度に、左右独立の時定数付きランダムウォーク（時定数 2s、±8% でクランプ）で
スリップ率 `slipL`/`slipR` を持たせています。CAN 配信 (`publishVehicleInfoCan`) とオドメトリ推定
(`integrateLocalizer`) は `measuredWheelSpeeds()`（スリップ込みの計測値）を使う一方、物理演算・当たり判定・
3D 描画は従来通り真の `wheelSpeeds()` のままです。これにより、`physics.x/y/yaw`（真の位置）と
`localizer.x/y/yaw`（`LaneNavigator` に渡る推定位置、HUD「詳細」タブの「速度 X/Y」「角速度 Z」のもと）が
走行中に少しずつ乖離していきます（`window.__sim.physics` と `window.__sim.localizer` を比べると確認できます）。
コーンランドマーク補正 (`coneLandmarkCorrection()`) は、この乖離を打ち消す材料として使われます。
HUD「詳細」タブのスリップ L/R 表示は現在値のデバッグ用です。

## HUD

- **常時表示（上部）**: 速度・旋回、位置 x/y（オドメトリ）・向き、コース内/逸脱中と逸脱回数、接触中。
  ヘッダのバッジは今の指令の出どころ（手動 / 自動運転 / 待機）です。
- **走行**: WASD のキー表示、「スタート位置へ」、視点リセット・視点回転
- **自動運転**: 自動運転 ON/OFF、走行状態・周回・検出器・メッセージ、白線検出の切替、1 周目を終える・記録リセット
- **詳細**: オドメトリ速度、IMU、CAN 車輪速、twist_mux、周回マップの境界断面、rosbridge 接続設定

右側の 3 枚は、上から機体カメラ・白線検知・周回マップです。

## 既知の制約

- 読み込むメッシュは車体 (`AIF_body.dae`) と車輪 (`tire.dae`) のみです。
  ZED カメラの `zedx.stl`（約13MB）は読み込み時間短縮のため対象外にしています。
- 障害物（MyLaps ゲートの支柱とコーン）には当たり判定があり、コース逸脱は HUD で検知しますが、
  コースの他の部分（内側の縁石・島など）には当たり判定がありません。
- ゲートは中央線を走る周回だけを止め、中央線から離れて走るレーシングラインは素通りします
  （「自動運転での確認結果」を参照）。
