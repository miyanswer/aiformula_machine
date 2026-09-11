# 05. 起動手順 & Launchファイル構成

実機運用時およびシミュレーション時におけるノードの起動手順、Launchファイルの階層関係、便利なシェルスクリプトの一覧です。

---

## 🚀 1. 実機での基本起動フロー

```mermaid
sequenceDiagram
    autonumber
    actor User as 開発者 / オペレーター
    participant CAN as Linux SocketCAN (can0)
    participant HW as ZED X / VectorNav
    participant Launch as all_robot_nodes.launch.py
    participant Nav as oit_navigation (run.launch.py)

    User->>CAN: 1. can_bringup.sh (500kbps 起動)
    User->>HW: 2. init_sensors.sh (センサ権限・接続確認)
    User->>Launch: 3. ros2 launch sample_launchers all_robot_nodes.launch.py
    Launch->>Launch: センシング・TF・認識・twist_mux・motor_controller 起動
    User->>Nav: 4. ros2 launch oit_navigation run.launch.py
    Nav->>Nav: multi_line_follower & obstacle_avoider 起動 (自律走行開始)
```

---

## 🛠️ 2. コマンドライン手順

### ① CAN バスの初期化
```bash
# SocketCAN インターフェース (can0) を 500kbps / 1Mbps でアップ
sudo ip link set can0 up type can bitrate 500000
# または提供されているシェルスクリプトを実行:
bash src/launchers/sample_launchers/shellscript/can_bringup.sh
```

### ② 全体システム（センシング・認識・制御・TF）の起動
```bash
ros2 launch sample_launchers all_robot_nodes.launch.py
```
- **オプション引数**:
  - `autopilot:=true` : 自動操縦モードで起動
  - `use_sim_time:=true` : rosbag 再生時などのシミュレーション時刻を使用

### ③ ナビゲーション（自律レーン追従・障害物回避）の起動
```bash
ros2 launch oit_navigation run.launch.py
```

### ④ 手動ジョイスティック操縦（テスト時）
```bash
ros2 launch sample_launchers gamepad_teleop.launch.py
```

### ⑤ 走行データの rosbag 記録
```bash
bash src/launchers/sample_launchers/shellscript/record_rosbag.sh
```
カメラ画像、オドメトリ、CANフレーム、制御指令などがタイムスタンプ付きで自動記録されます。

---

## 📂 3. Launchファイル階層構造

```
all_robot_nodes.launch.py (統合エントリポイント)
├── vehicle_tf_broadcaster.launch.py   # 車両URDF/TFブロードキャスト
├── zedx_camera.launch.py              # ZED X ステレオカメラ
├── vectornav.launch.py                # VectorNav IMU/GNSS
├── socket_can_bridge.launch.py        # SocketCAN <-> ROS 2 CAN Frame
├── object_road_detector.launch.py     # YOLOP 道路/コーン認識
├── lane_line_publisher.launch.py      # 白線3次スプラインフィッティング
├── object_publisher.launch.py         # 3D物体位置同定
├── twist_mux.launch.py                # 速度指令多重化 & 安全ロック
└── motor_controller.launch.py         # 差動2輪 RPM変換 & CAN送信
```

---

## 📊 4. 便利な設定ファイル (YAML)

| 設定ファイルパス | 内容 |
| :--- | :--- |
| `launchers/sample_launchers/config/topic_list.yaml` | 全トピック名の一元管理 |
| `launchers/sample_launchers/config/frame_id_list.yaml` | 全TFフレーム名の一元管理 |
| `launchers/sample_launchers/config/twist_mux.yaml` | 速度指令の優先順位とタイムアウト設定 |
| `vehicles/sample_vehicle/config/wheel.yaml` | 車輪径、トレッド幅、ギヤ比パラメータ |
| `perception/lane_line_publisher/config/lane_line_publisher.yaml` | レーン抽出スキャン範囲、多項式フィッティング次数 |
