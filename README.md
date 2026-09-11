# aiformula_machine (AI Formula 車載自律走行・機体プラットフォーム)

AI Formula 実車機体に搭載する **機体ハードウェア基盤（センシング・モーター駆動・安全機構）** と **自律走行スタック（白線認識・2D BEV 追従制御・信号機距離推定）** を統合したパッケージ群です。

実機上での自律走行はもちろん、実機がない環境でも **PC 単体（Docker / 動画入力）でアルゴリズム検証・可視化（RViz2 / Web GUI）** を即座に行うことができます。

---

## 🏎️ 含まれるパッケージ・ディレクトリ構成

```
aiformula_machine/
├── src/
│   └── oit_navigation/       # 自律走行スタック (YOLOP白線認識, 2D BEV Pure Pursuit制御, 信号機距離推定)
├── sensing/
│   ├── zed-ros2-wrapper/     # ZED X ステレオカメラ 公式ラッパー (RGB / Depth / IMU)
│   ├── vectornav/            # VectorNav 9軸IMU / GNSS ドライバ
│   ├── odometry_publisher/   # 車輪エンコーダ ＋ ジャイロ統合オドメトリ
│   └── rear_potentiometer/   # ステアリング・後輪角度センサ
├── control/
│   ├── motor_controller/     # 速度指令 Twist を左右車輪 RPM に変換し CAN 送信 (ID: 0x210)
│   └── twist_mux/            # 安全優先度マルチプレクサ (非常停止・手動介入優先切替)
├── vehicles/
│   └── sample_vehicle/       # 車両の物理モデル、URDF/Xacro、TF座標系、ZED Xカメラマウント位置
├── launchers/
│   └── sample_launchers/     # 機体一括起動 Launch (hardware_bringup, all_system_2027)
├── common/
│   ├── aiformula_interfaces/ # 共通ROS2メッセージ・サービス型定義
│   ├── common_python/        # Python 共通ユーティリティ
│   ├── common_cpp/           # C++ 共通ユーティリティ
│   └── serial/               # シリアル通信ライブラリ
├── bash/                     # 起動用シェルスクリプト群
├── docker/ & compose.yaml    # Docker 環境定義 (ROS 2 Humble, noVNC/RViz2, Web GUI)
├── models/                   # AI推論用学習済み重みモデル (.pth, .pt)
└── docs/                     # アーキテクチャ・設計仕様詳細ドキュメント
```

| ディレクトリ / パッケージ | 役割・説明 |
| :--- | :--- |
| **`src/oit_navigation`** | **自律走行スタック**: YOLOP白線認識・2D BEV変換・スライディングウィンドウ追跡・Pure Pursuit制御・信号機距離推定・Web検証GUI |
| **`vehicles/sample_vehicle`** | 車両物理モデル、URDF/Xacro、TF座標系定義、ZED Xマウント位置 |
| **`sensing/zed-ros2-wrapper`**| ZED X ステレオカメラ 公式ドライバ (RGB/Depth/Point Cloud/IMU) |
| **`sensing/vectornav`** | VectorNav 9軸IMU / GNSS ドライバ |
| **`sensing/odometry_publisher`** | 車輪エンコーダ ＋ ジャイロ統合オドメトリ配信 |
| **`sensing/rear_potentiometer`** | ステアリング・後輪舵角センサドライバ |
| **`control/motor_controller`** | 速度指令 `Twist` を左右車輪 RPM に変換し CAN 送信 (ID: `0x210`) |
| **`control/twist_mux`** | 安全優先度マルチプレクサ (非常停止・手動介入を最優先に切替) |
| **`launchers/sample_launchers`**| 機体一括起動 Launch (`hardware_bringup.launch.py`, `all_system_2027.launch.py`) |
| **`common/aiformula_interfaces`**| 機体共通のカスタムメッセージ型・サービス定義 |
| **`bash/`** | 各種起動ワンライナースクリプト群 |

---

## 💻 開発・実行ワークフロー

### 1. Docker 環境のセットアップ（PC単体開発・Mac / Linux / Windows）

本リポジトリは Docker Compose を利用して、ホストOSを汚さずに ROS 2 Humble 環境と GUI ツールを起動できます。

```bash
# コンテナのビルド & 起動 (バックグラウンド)
docker compose up -d

# コンテナのシェルに入る場合
docker compose exec aiformula_ws bash
```

> **🌐 ブラウザでアクセス可能な Web UI:**
> - **RViz2 / noVNC 画面:** [http://localhost:8080](http://localhost:8080)
> - **動画検証 Web GUI:** [http://localhost:8090](http://localhost:8090)

---

### 2. PC 単体での動作確認・アルゴリズム検証（実機不要）

実機がなくても、車載カメラの録画動画（MP4）を再生して白線認識・走行ライン生成・信号機検出・RViz2 可視化を PC 単体でテストできます。

#### 【方法 A】Web 検証 GUI を使う（おすすめ）
ブラウザ上で動画選択、検証パイプライン（白線単体 / 信号機単体 / 統合制御）の選択、起動・停止を直感的に行えます。

```bash
# Docker コンテナ内で実行 (またはホスト側で 2_test_pc_standalone.sh 実行)
ros2 run oit_navigation verification_gui
```
👉 ホスト PC のブラウザで [http://localhost:8090](http://localhost:8090) を開いて操作します。

#### 【方法 B】ワンライナースクリプトで起動
```bash
# ルート直下から実行 (Docker環境でも自動認識して実行されます)
./2_test_pc_standalone.sh

# 任意の動画パスやデバイスを指定する場合:
./2_test_pc_standalone.sh /aiformula_ws/mp4/custom_video.mp4 cpu
```
👉 ブラウザで [http://localhost:8080](http://localhost:8080) を開くと、リアルタイムに白線認識結果や緑色の目標走行ラインが RViz2 に描画されます。

---

### 3. 実機（実車機体）での運用手順

実機（Jetson / 車載PC）上で実行する際の手順です。

#### A. 機体ハードウェアのみを起動（外部指令待機状態）
センサー（ZED X, IMU, CAN 等）と安全機構を初期化し、速度指令トピック待機状態にします。
```bash
bash bash/1_bringup_hardware.sh
```

#### B. 実機フルシステム（機体 ＋ 自律走行）を一括起動
ハードウェア初期化から、YOLOP 白線認識・BEV 追従制御・信号機検知までの全ノードを 1 コマンドで起動します。
```bash
bash bash/3_bringup_all_nodes.sh
```

#### C. キーボード手動操縦（動作確認・キャリブレーション用）
`twist_mux` 経由で最優先で手動操作を行います。
```bash
bash bash/teleop_keyboard.sh
```

---

## 📚 各機能の詳細ドキュメント

- **自律走行・白線認識・追従制御・信号機検知の詳細**:
  - [src/oit_navigation/README.md](file:///Users/miyanswer/aiformula_machine/src/oit_navigation/README.md)
- **機体アーキテクチャ・トピック仕様・詳細設計**:
  - [docs/reference_ai_formula_oit_2026/](file:///Users/miyanswer/aiformula_machine/docs/reference_ai_formula_oit_2026/)
    - `01_system_architecture.md`: システム全体構成・ノード連携
    - `02_topic_and_interfaces.md`: トピック名・型・インターフェース仕様
    - `03_perception_and_ai.md`: 白線認識・信号機認識アルゴリズム
    - `04_navigation_and_control.md`: 2D BEV 幾何変換・Pure Pursuit 制御理論
    - `05_launch_and_operations.md`: Launch構成・運用手順
