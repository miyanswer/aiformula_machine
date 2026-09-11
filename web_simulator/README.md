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

ROS 2 (Humble) 環境で、`rosbridge_server` パッケージが未導入なら先にインストールします
（この Docker イメージには現状含まれていません）。

```bash
sudo apt-get update && sudo apt-get install -y ros-humble-rosbridge-server
```

ROS 2 環境を source した状態で起動します。

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
`geometry_msgs/msg/Twist` を 20Hz で publish し続けます（速度 0 の Twist が流れるので、
twist_mux 側のタイムアウトでロックされることもありません）。

### 4. トピックが流れているか確認する

rosbridge が起動しているマシン（コンテナ内）で、ROS 2 環境を source して確認します。

```bash
source /opt/ros/humble/setup.bash

ros2 topic list                                          # トピックが出現しているか
ros2 topic echo /aiformula_control/gamepad/cmd_vel        # 実際に Twist が流れているか
ros2 topic hz /aiformula_control/gamepad/cmd_vel          # 20Hz 前後で来ているか
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

画面右上には、車体に搭載された ZED カメラ位置（`config/zedx/extrinsic/extrinsic.yaml`
の取り付けオフセット）から前方を見た**機体カメラ視点**をピクチャーインピクチャーで
常時表示します。

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
│   └── vehicle_physics.js   差動2輪駆動の物理モデル（質量70kg）
└── vendor/                  three.js / roslib.js のローカル同梱コピー
```

## 既知の制約

- 読み込むメッシュは車体 (`AIF_body.dae`) と車輪 (`tire.dae`) のみです。
  ZED カメラの `zedx.stl`（約13MB）は読み込み時間短縮のため対象外にしています。
- 衝突判定・地形は未実装です（平面のグラウンドのみ）。
