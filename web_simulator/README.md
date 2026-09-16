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
使う理由: 「モデル (ONNX)」検知モードの onnxruntime-web が要求する COOP/COEP
ヘッダーを付与するためです。詳しくは後述の「実装上の注意」を参照してください。）

ブラウザで `http://localhost:8000/web_simulator/` を開きます。車体モデルが表示され、
WASD で動かせれば起動成功です（この時点では rosbridge 未接続でも問題ありません）。

Three.js / roslib.js はオフライン環境でも動くよう `vendor/` 配下にローカル同梱済みです
（追加のビルドや `npm install` は不要）。

**対応ブラウザ**: Chrome/Edge に加えて **Firefox でも動作確認済み**です
（WebGL描画、閾値処理モード、モデル(ONNX)モード、HUDのタブ切替・折りたたみ・ドラッグ移動、
いずれもエラーなし）。「モデル (ONNX)」モードは WebGPU が使えない環境（`navigator.gpu` が
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

## oit_navigation 白線検知・BEV変換・Pure Pursuit

`src/oit_navigation` パッケージが実車で行っている **白線検知 → 2D BEV（鳥瞰図）
変換 → スライディングウィンドウ白線追跡 → Pure Pursuit 操舵制御** を、そのまま
ブラウザ内（[`js/lane_threshold_detector.js`](js/lane_threshold_detector.js) /
[`js/lane_model_detector.js`](js/lane_model_detector.js) /
[`js/oit_lane_pipeline.js`](js/oit_lane_pipeline.js)）に移植しています。

白線検知は HUD の「検知方式」ボタンで以下の2方式をいつでも切り替えられます
（切り替えてもBEV変換・Pure Pursuit以降のロジックは共通です）。

- **閾値処理**（既定）: 輝度＋彩度の閾値（明るくかつグレー/白に近い＝彩度が低い
  ピクセルを白線とみなす）で判定。モデルの読み込みや推論待ちが不要で、キャプチャ
  した毎フレームに対して同期的に実行されます。
- **モデル (ONNX)**: 実際に学習済みの白線セグメンテーションモデル
  （`models/honda_shihou_finetuned_best.pth`）を ONNX エクスポートし、
  [onnxruntime-web](vendor/onnxruntime-web) の **WebGPU バックエンド**でブラウザの
  GPU上で推論します（`navigator.gpu` が無い環境では自動的に CPU の WASM
  バックエンドへフォールバック）。初めてこのモードに切り替えた時に遅延読み込み
  されます（既定の閾値処理モードでは読み込まれません）。読み込み・推論に失敗
  した場合は自動的に閾値処理モードへフォールバックします。
  - 前処理は実車の `yolop_lane_detector.py` の `roi_mode: crop_bottom` と同じ
    考え方で、機体カメラ画像の上部（空など走路と無関係な領域、`top_cut_ratio`
    比率分）を切り捨て、残った下部を **そのまま 640×640 の正方形にリサイズ**
    してモデルに入力します（レターボックスの余白なし）。

画面右上の機体カメラ視点の下に、2枚のパネルが並びます。

- **白線検知**: 機体カメラ画像に、検出した白線マスクを緑半透明でオーバーレイ
  （`yolop_lane_detector.py` の `draw_lane_lines` と同じ 50% アルファブレンド。
  検知方式を切り替えてもこのオーバーレイ自体の描き方は変えていません）
- **BEV / Pure Pursuit**: 白線マスクを 2D 俯瞰座標系に変換し、スライディング
  ウィンドウで左（水色）／右（黄色）白線を追跡。緑の目標ラインは Pure Pursuit が
  実際に追従する **原点アンカー付き軌道**（`bev_pure_pursuit_node.py` の
  `_run_control_step` が計算する Hermite スプライン）で、自車位置（EGO マーカー）
  から必ず接続した状態で描画されます。緑の丸は実際に操舵計算で使っている
  lookahead 点（5m 先）です。

HUD の「自動運転」ボタンを ON にすると、この Pure Pursuit の操舵計算（実車の
`bev_pure_pursuit_node.py` の `_run_control_step` と同じ式）で実際に車両が走行します。
最高速度・最大旋回角速度は、実車の `navigation_params.yaml` の値
（`target_linear_speed=1.0`, `max_angular_speed=1.5`）ではなく、**本シミュレータの
WASD 走行と同じ上限**（1.5 m/s / 1.2 rad/s、下記「車体モデル・物理パラメータ」参照）
を使うよう指示により固定しています。

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

実車の `oit_navigation` と同じトピック名・型で配信します（rosbridge 接続中のみ）。
`config/navigation_params.yaml` / 各ノードのパラメータデフォルト値と完全に一致させて
あるので、RViz2 など既存の可視化・購読側の設定をそのまま流用できます。

| トピック名 | メッセージ型 | 内容 |
| :--- | :--- | :--- |
| `/aiformula_perception/object_road_detector/mask_image` | `sensor_msgs/msg/Image` (mono8) | 白線マスク（検知方式に応じて閾値処理/モデル） |
| `/aiformula_visualization/object_road_detector/annotated_image` | `sensor_msgs/msg/Image` (rgb8) | カメラ+緑オーバーレイ |
| `/aiformula_visualization/bev_annotated_image` | `sensor_msgs/msg/Image` (rgb8) | BEV 可視化画像 |
| `/aiformula_perception/lane_line_publisher/annotated_mask_image` | `sensor_msgs/msg/Image` (rgb8) | BEV 可視化画像（複製） |
| `/aiformula_perception/lane_line_publisher/lane_lines/{left,right,center}` | `nav_msgs/msg/Path` | 検出白線・目標ラインの点列 |
| `/aiformula_visualization/target_trajectory` | `nav_msgs/msg/Path` | 原点アンカー付き目標軌道 |
| `/aiformula_control/extremum_seeking_mpc/cmd_vel` | `geometry_msgs/msg/Twist` | 自動運転の操舵・速度指令（twist_mux の "mpc" 入力） |
| `/aiformula_control/lane_tracker/status` | `std_msgs/msg/String` | モード・速度・曲率ステータス |
| `/aiformula_control/twist_mux/cmd_vel` | `geometry_msgs/msg/Twist` | twist_mux 調停後の指令値（"自動運転" ON時のみ mpc が混ざる） |

### 白線検知の閾値パラメータ（閾値処理モード）

[`js/lane_threshold_detector.js`](js/lane_threshold_detector.js) の
`LUMA_THRESHOLD`（輝度、既定 175/255）・`MAX_CHROMA`（彩度、既定 40）・
`TOP_CUT_RATIO`（画面上部の除外比率、既定 0.45）で調整できます。コースの照明や
路面色を変えた場合はここを調整してください。

### ONNX モデルの再エクスポート（モデルモード）

重み (`models/honda_shihou_finetuned_best.pth`) を更新した場合は、以下で
`web_simulator/models/honda_shihou_finetuned.onnx` を再生成してください
（object-detection ヘッドはブラウザ側で使わないため、白線セグメンテーション
ヘッドのみエクスポートします）。

```bash
python3 src/oit_navigation/oit_navigation/export_onnx_web.py \
  --weights models/honda_shihou_finetuned_best.pth \
  --output web_simulator/models/honda_shihou_finetuned.onnx
```

入力サイズは 640×640 の正方形で固定エクスポートしています（`js/lane_model_detector.js`
の crop_bottom 前処理が常にこのサイズへリサイズするため）。別サイズにする場合は
`--size` と `js/lane_model_detector.js` の `MODEL_INPUT_SIZE` を合わせて変更してください。

### 実装上の注意

- 「モデル (ONNX)」モードは [`js/lane_model_detector.js`](js/lane_model_detector.js)
  で `executionProviders: ['webgpu']` を指定し、onnxruntime-web の WebGPU
  バックエンド（`vendor/onnxruntime-web/ort.webgpu.min.js`）でブラウザの GPU上
  推論します。以前は WASM (CPU) バックエンドのみだったため、重いモデルだと
  推論のたびにメインスレッド（描画スレッドと同じ）がブロックされ描画がカクつい
  ていました。WebGPU が使えない環境（`navigator.gpu` が無い、または
  `InferenceSession.create` が失敗する場合）は自動的に CPU の WASM バックエンド
  へフォールバックします。
- onnxruntime-web 1.19 以降、同梱する wasm バイナリはスレッド版
  （`SharedArrayBuffer` 利用）のみで、非スレッド版は配布されなくなりました。
  `SharedArrayBuffer` はページが cross-origin isolated（`COOP: same-origin` +
  `COEP: require-corp`）である場合のみ有効になるため、`webgpu` バックエンドは
  もちろん、CPU フォールバックの `wasm` バックエンドも含めてこのヘッダーが
  必須です。これが起動手順で `python3 -m http.server` の代わりに
  [`web_simulator/serve.py`](serve.py) を使う理由です。ヘッダーなしで配信すると
  （例: 別の静的サーバーやファイル直開き）「モデル (ONNX)」モードの読み込みが
  失敗し、自動的に閾値処理モードへフォールバックします。
- [`js/lane_model_detector.js`](js/lane_model_detector.js) の
  `ort.env.wasm.wasmPaths` には相対パスではなく絶対 URL
  （`new URL(wasmDir, document.baseURI).href`）を渡しています。onnxruntime-web
  は wasm/mjs グルーコードを `import()` で動的解決するため、`"vendor/..."` の
  ようなスキームなし・`./`なしの相対パスは無効な bare module specifier と
  みなされ、`webgpu`・`wasm` の両バックエンドともロードに失敗します。
- `yolop_lane_detector.py` は cv2/BGR 画像を RGB 用の ImageNet mean/std へそのまま
  正規化しており（RGB 変換なし）、`honda_shihou_finetuned_best.pth` も同じ
  パイプラインでファインチューニングされています。ブラウザ側の Canvas は RGB で
  取得されるため、[`js/lane_model_detector.js`](js/lane_model_detector.js) は
  意図的にチャンネル順を B/G/R に入れ替えてから正規化し、実車と同じ数値を再現して
  います。
- コース地面（[`png/shihou_cource_unity.png`](png/shihou_cource_unity.png)）は
  実写ではなく Unity 上のコース設計ツールのスクリーンショットですが、白線が
  グレー路面に対しはっきりした白色で描かれているため、どちらの検知方式でも
  問題なく白線として検出できています。

### ROS 2連携モード（重いYOLOP/BEV/Pure Pursuit計算をROS 2側で実行）

ブラウザ内ONNXモデルは WebGPU が使える環境では GPU 上で推論しますが、閾値処理は
常にメインスレッド上で同期実行ですし、WebGPU が使えない環境ではモデルも CPU
(WASM) にフォールバックするため、それでも描画がカクつくことがあります。
「検知方式」に **ROS2連携** を選ぶと、YOLOP 推論と BEV変換・Pure Pursuit 制御を
実車と同じ ROS 2 ノード
（GPU/Dockerコンテナ側）で実行し、rosbridge_server 経由でその結果をブラウザに
表示・車両制御に反映します（ブラウザ側の計算は一切行いません）。

#### 使い方

```bash
# 1. rosbridge_server を起動（ホスト側）
make rosbridge

# 2. 認識・制御パイプラインを起動（別ターミナル、初回のみビルドが必要）
make bash
# コンテナ内で:
colcon build --packages-select oit_navigation --symlink-install
source install/setup.bash
ros2 launch oit_navigation simulator_test.launch.py
# GPUがあれば use_device:=0 を付けるとYOLOP推論が高速化されます
```

3. ブラウザで `http://localhost:8000/web_simulator/` を開き「接続」→
   「検知方式」で **ROS2連携** を選択 → 「自動運転」を ON にすると、
   ROS 2 側が計算した経路でコースを自律走行します。

`simulator_test.launch.py`（[`src/oit_navigation/launch/simulator_test.launch.py`](../src/oit_navigation/launch/simulator_test.launch.py)）
は `video_test.launch.py` から動画再生ノード (`video_publisher`) を除いたもので、
カメラ画像の入力は Web シミュレータが rosbridge 経由で配信する圧縮画像
（下記トピック表参照）です。`yolop_lane_detector` / `bev_pure_pursuit_node` の
パラメータは実車と同じ `config/navigation_params.yaml` をそのまま使います。

#### 双方向トピック一覧

| 方向 | トピック名 | メッセージ型 | 内容 |
| :--- | :--- | :--- | :--- |
| Sim→ROS | `/aiformula_sensing/zed_node/left_image/undistorted/compressed` | `CompressedImage` | 機体カメラ映像（yolop_lane_detectorの入力） |
| Sim→ROS | `/aiformula_sensing/vectornav/imu` | `Imu` | IMU |
| Sim→ROS | `/aiformula_sensing/gyro_odometry_publisher/odom` | `Odometry` | オドメトリ |
| ROS→Sim | `/aiformula_perception/object_road_detector/mask_image` | `Image` (mono8) | YOLOP 白線マスク → 白線検知パネルへ |
| ROS→Sim | `/aiformula_visualization/bev_annotated_image` | `Image` (bgr8) | BEV可視化画像（そのままBEVパネルへ表示） |
| ROS→Sim | `/aiformula_control/extremum_seeking_mpc/cmd_vel` | `Twist` | Pure Pursuit 制御指令 → `twistMux.update('mpc', ...)` |
| ROS→Sim | `/aiformula_control/lane_tracker/status` | `String` | モード文字列 → 「モード」HUD表示 |

> **トピック名について**: 上記はすべて実車の `topic_list.yaml` /
> `navigation_params.yaml` にある名前をそのまま使っています（Webシミュレータの
> 他の機能もすべて同じ名前で配信・購読済みです）。制御指令は
> `/cmd_vel` や独自の `target_twist` ではなく、実車の twist_mux が
> "mpc" 入力として扱う `/aiformula_control/extremum_seeking_mpc/cmd_vel` を
> そのまま使うことで、[twist_mux（WASD と自動運転の調停）](#twist_muxwasd-と自動運転の調停)
> の "mpc" ソースにも変更なくそのまま流し込めます。

#### 実装

- [`js/simulator.js`](js/simulator.js) の `onRosMaskImage` / `onRosBevAnnotatedImage` /
  `onRosAutonomousCmdVel` / `onRosLaneTrackerStatus` が上記トピックの購読処理です。
  `detectorMode === 'ros2'` の間だけ動作し、ブラウザ内 `updateLanePipeline()`
  （閾値処理/ONNXモデル）は完全に停止します。
- `mask_image` はブラウザ側の検出結果と同じ形式（mono8 の 0/255 マスク）なので、
  既存の `applyLaneOverlay()`（緑オーバーレイ描画）をそのまま再利用しています。
- `bev_annotated_image` は `bev_pure_pursuit_node` が描画済みの画像
  （スライディングウィンドウ・左右白線・目標ライン・EGOマーカー入り）を
  そのまま Canvas に表示するだけで、ブラウザ側での再計算はしていません。
- `extremum_seeking_mpc/cmd_vel` は受信したら即座に `twistMux.update('mpc', v, omega, now)`
  へ渡すだけで、ブラウザ側で再計算はしません。ROS 2 側が 0.3 秒（twist_mux の
  mpc タイムアウト）以上メッセージを止めれば、`js/twist_mux.js` が自動的に
  タイムアウトと判断し、WASD操作や停止に切り替わります（実車と同じ挙動）。

#### 既知の制約

- `use_device:=cpu` では YOLOP 推論に数百ms かかることがあり、twist_mux の
  mpc タイムアウト（0.3秒）より遅くなると、指令が瞬間的に途切れて速度が
  一瞬コースト（惰性減速）することがあります。滑らかに走らせたい場合は GPU
  環境で `use_device:=0` を指定してください。
- `sensor_msgs/msg/Image` の `data` フィールドは rosbridge が自動で
  base64文字列に変換して送受信するため、型不一致やデコードエラーは
  発生しません（[`js/simulator.js`](js/simulator.js) の `base64ToUint8Array`
  が既存の `uint8ArrayToBase64` と対になっています）。
- 閾値処理モード・ブラウザ内モデルモードでの手動WASD走行はこれまで通り
  ブラウザ単体で動作し、rosbridge/ROS 2側が起動していなくても壊れません。

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
│   ├── lane_threshold_detector.js  閾値処理による白線マスク検出
│   ├── lane_model_detector.js      ONNXモデルによる白線マスク検出（crop_bottom前処理）
│   ├── oit_lane_pipeline.js BEV変換・スライディングウィンドウ白線追跡・Pure Pursuit
│   └── twist_mux.js         WASD/自動運転の優先度＋タイムアウト調停
├── models/                  honda_shihou_finetuned.onnx（export_onnx_web.py の出力）
└── vendor/                  three.js / roslib.js / onnxruntime-web のローカル同梱コピー
```

## コースレイアウト

ユーザー提供のコースレイアウト画像（[`png/shihou_cource_unity.png`](png/shihou_cource_unity.png)、
Unity上のコース設計ツールのスクリーンショット）を、[`js/course.js`](js/course.js) で
そのまま地面テクスチャとして読み込み、車体の初期位置を中心に敷いています。

- 実寸法（外形約98.9m×91.4m、道幅9.0mなど）は現時点では未反映です（指示によりいったん保留）。
  地面の大きさは画像のアスペクト比（1024:819）だけを使い、幅100m相当で仮に配置しています
- 画像をそのまま貼っているだけの **見た目のみ** の実装です。道路外への進入判定や
  ゲート通過判定、周回計測などのロジックは実装していません（車体はこれまで通り
  どこでも自由に走行できます）
- 実寸法を反映する場合は `js/course.js` の `COURSE_WIDTH_M`（現在100m固定）を
  実際のコース幅に合わせて変更してください
- 車体の初期位置は odom 原点 `(0,0,0)` のままです。コース画像は
  [`js/simulator.js`](js/simulator.js) の `COURSE_POSE` で現在
  `(x: 13.22, y: 35.41, yaw: +90°)` に配置しています

## 既知の制約

- 読み込むメッシュは車体 (`AIF_body.dae`) と車輪 (`tire.dae`) のみです。
  ZED カメラの `zedx.stl`（約13MB）は読み込み時間短縮のため対象外にしています。
- 衝突判定・地形は未実装です（コースの路面はテクスチャのみで、道路外進入の制限はありません）。
