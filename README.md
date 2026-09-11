# aiformula_base (機体ハードウェア基盤 & プラットフォーム層)

AI Formula 実車機体を動かすための **提供ベースフレームワーク（ハードウェア抽象化・センシング・モーター駆動・安全機構）** をまとめた最小構成パッケージ群です。

旧Perception（306MB）や不要なアルゴリズム層を排除し、**純粋な機体プラットフォーム（約74MB）** にスリム化されています。

---

## 🏎️ 含まれるパッケージ一覧

| カテゴリ | パッケージ名 | 役割・説明 |
| :--- | :--- | :--- |
| **`vehicles/`** | `sample_vehicle` | 車両の物理モデル、URDF/Xacro、TF座標系、ZED Xカメラマウント位置 |
| **`sensing/`** | `zed-ros2-wrapper` | ZED X ステレオカメラ 公式ラッパー (RGB / Depth / IMU) |
| | `vectornav` | VectorNav 9軸IMU / GNSS ドライバ |
| | `odometry_publisher` | 車輪エンコーダ ＋ ジャイロ統合オドメトリ |
| | `rear_potentiometer` | ステアリング・後輪角度センサ |
| **`control/`** | `motor_controller` | 速度指令 `Twist` を左右車輪 RPM に変換し CAN 送信 (ID: `0x210`) |
| | `twist_mux` | 安全優先度マルチプレクサ (非常停止・手動介入優先切替) |
| **`launchers/`**| `sample_launchers` | 機体一括起動 Launch (`hardware_bringup.launch.py`) |
| **`common/`** | `aiformula_interfaces` | 共通メッセージ型定義 |
| | `common_python`, `common_cpp` | 共通Launchユーティリティ |
| **`bash/`** | `1_bringup_hardware.sh` | [実機専用] 機体・センサー類・モーター駆動・安全機構の一括起動 |
| | `2_test_pc_standalone.sh` | [PC単体検証] 実機なしでPC単体でアルゴリズム/新機能の検証・開発を行う |
| | `3_bringup_all_nodes.sh` | [実機専用] 機体ハードウェア＋自律走行（2027システム）の全ノード一括起動 |
| | `teleop_keyboard.sh` | キーボードによる手動操縦（Twist Mux連携） |

---

## 💻 開発・実行ワークフロー

### A. 実機なしでPC単体で検証・開発する場合（推奨）
実機が手元になくても、カメラ動画（MP4）を擬似入力として、レーン検出・BEV追従制御・信号機認識・可視化（RViz2）をPC単体でテストできます。
```bash
cd /path/to/aiformula_ws
bash src/aiformula_base/bash/2_test_pc_standalone.sh

# 任意の動画やCPU実行を指定する場合:
bash src/aiformula_base/bash/2_test_pc_standalone.sh /path/to/video.mp4 cpu
```

---

### B. 実車で機体ハードウェアのみを起動する場合
実車上でCANやカメラ、IMUなどのハードウェア基盤を起動し、外部からの速度指令待機状態にします。
```bash
bash src/aiformula_base/bash/1_bringup_hardware.sh
```

---

### C. 実車で全システム（機体＋自律走行）を一括起動する場合
実車上でハードウェア初期化から、認識・制御・信号機検知までの全ノードを1コマンドで起動します。
```bash
bash src/aiformula_base/bash/3_bringup_all_nodes.sh
```

---

### D. キーボードによる手動操縦（動作確認用）
```bash
bash src/aiformula_base/bash/teleop_keyboard.sh
```
