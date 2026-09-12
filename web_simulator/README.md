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
python3 -m http.server 8000
```

ブラウザで `http://localhost:8000/web_simulator/` を開きます。車体モデルが表示され、
WASD で動かせれば起動成功です（この時点では rosbridge 未接続でも問題ありません）。

Three.js / roslib.js はオフライン環境でも動くよう `vendor/` 配下にローカル同梱済みです
（追加のビルドや `npm install` は不要）。

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
│   └── course.js            コースレイアウトの地面テクスチャ生成
└── vendor/                  three.js / roslib.js のローカル同梱コピー
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
