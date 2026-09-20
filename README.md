# aiformula_machine (AI Formula 車載自律走行・機体プラットフォーム)

AI Formula 実車機体に搭載する **機体ハードウェア基盤（センシング・モーター駆動・安全機構）** と **自律走行スタック（白線認識・周回マップ作成・QP レーシングライン走行・信号機距離推定）** を統合したパッケージ群です。

実機上での自律走行はもちろん、実機がない環境でも **PC 単体（Docker / 動画入力）でアルゴリズム検証・可視化（RViz2 / Web GUI）** を即座に行うことができます。

---

## 🏎️ 含まれるパッケージ・ディレクトリ構成

```
aiformula_machine/
├── src/
│   └── oit_navigation/       # 自律走行スタック (白線 左/中央/右 検出, 周回マップ + QP レーシングライン, 信号機距離推定)
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
| **`src/oit_navigation`** | **自律走行スタック**: 白線検出 (YOLOP 既定 / UFLD) で左境界・中央線・右境界を認識 → 1周目は中央白線トラッキング走行しながら左右境界を記録 → 2周目以降は QP (最小曲率) のアウト・イン・アウト経路を走行。信号機距離推定・Web検証GUI |
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

### 1. Docker 環境のセットアップ（Mac / Ubuntu / Windows 自動対応）

本リポジトリは `Makefile` により **Mac（Apple Silicon / CPU）** と **Ubuntu（NVIDIA GPU）** を自動検知して最適な環境を立ち上げます。

```bash
# コンテナのビルド (MacはCPU版、Ubuntu+GPUはCUDA版PyTorchを自動選択)
make build

# コンテナの起動 & シェルに入る
make bash
```

> **🖥️ 環境の自動判定について:**
> - **Mac (Apple Silicon / Intel)**: CPU モードで起動（手動設定不要）。
> - **Ubuntu (NVIDIA GPU)**: `nvidia-smi` を自動検知し、GPU パススルー（`compose.gpu.yaml`）および CUDA 12.1 対応 PyTorch でビルド・起動。
>   *(※ Ubuntu 側には `docker-ce` と `nvidia-container-toolkit` をインストールしておくだけでOKです)*

> **🤖 Jetson AGX Orin (JetPack 5.1.x / L4T R35) での実行:**
> - `make build` 実行前に `cat /etc/nv_tegra_release` で搭載中のL4Tバージョンを確認してください。`docker/Dockerfile.jetson` は既定で `r35.3.1`（JetPack 5.1.1相当）のベースイメージを使いますが、異なる場合は `make build JETSON_BASE_TAG=r35.2.1` のように上書きしてください（ズレると `torch.cuda.is_available()` が `False` になります）。
> - `docker info | grep -i runtime` で `nvidia` ランタイムが登録されていることを事前に確認してください（JetPack標準セットアップ済みであれば通常は有効です）。
> - `models/*_rtx_2070_..._sm75.engine` はRTX2070(sm75)向けのTensorRTエンジンで、Orin(sm87)では使われません。`lane_detector` (backend=yolop, `use_tensorrt:=true`) は起動時に現在のGPU向けのエンジンが無ければ自動でコンパイルし直すため（`src/oit_navigation/oit_navigation/yolop_lane_backend.py` の `_init_tensorrt` 参照）、追加の手動作業は不要ですが、初回起動時は数分ほど余分に時間がかかります。
> - 実機は GitHub から clone したリポジトリで走らせるため、白線検出は git 管理されている YOLOP の重み (`models/honda_shihou_finetuned_best.pth`) を使います (`backend:=yolop`, 既定)。UFLD の重み (245MB) は git 管理外です。
> - Jetsonでは `rviz_aiformula_plugins` パッケージ（RViz専用プラグイン、実車走行には不要）はビルド対象から外れます。`make build-ws` / `make build-pkg` が `IS_JETSON` を自動検知して `--packages-skip rviz_aiformula_plugins` を付与するため、いつも通り `make build-ws` を実行するだけで構いません（手動でフラグを付ける必要はありません）。
> - Jetson上ではRViz2/rqt本体をインストールしていない（ヘッドレス構成の）ため、`make rqt` / `make rqt-graph` / `make open-rviz` は動作しません。可視化が必要な場合はMac側の Web シミュレータ（[http://localhost:8000/web_simulator/](http://localhost:8000/web_simulator/)）や、動画検証用の Web GUI（PC単体検証時）を利用してください。
> - `sensing/zed-ros2-wrapper` ディレクトリには `COLCON_IGNORE` が置かれていないため、`colcon build` がこれもビルド対象に含めてしまい、ZED SDK が無い環境（Jetson/PC問わず）ではその分の失敗ログが出ることがあります（x86版でも既存の問題で、本ブランチが持ち込んだものではありません）。実車走行に `zed-ros2-wrapper` 自体は不要なので、失敗しても無視して構いません。

> **🌐 ブラウザでアクセス可能な Web UI:**
> - **3D 走行シミュレータ:** [http://localhost:8000/web_simulator/](http://localhost:8000/web_simulator/)
> - **RViz2 / noVNC 画面:** [http://localhost:8080](http://localhost:8080)
> - **動画検証 Web GUI:** [http://localhost:8090](http://localhost:8090)

---

### 2. PC 単体での動作確認・アルゴリズム検証（実機不要）

実機がなくても、車載カメラの録画動画（MP4）を再生して白線検出（左境界/中央線/右境界の割り当てまで）・信号機検出・RViz2 可視化を PC 単体でテストできます。
動画にはオドメトリが無いため、周回マップ作成と QP 走行は Web シミュレータ（`web_simulator/`、「理想検出/YOLOP/UFLD/ROS2連携」モード）で検証します。

#### 【方法 A】Web 検証 GUI を使う（おすすめ）
ブラウザ上で動画選択、検証パイプライン（信号機単体 / 白線検出 YOLOP / 白線検出 UFLD / 統合）の選択、起動・停止を直感的に行えます。

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
./2_test_pc_standalone.sh /aiformula_machine/mp4/custom_video.mp4 cpu
```
👉 ブラウザで [http://localhost:8080](http://localhost:8080) を開くと、リアルタイムに白線検出結果（左=水色/中央=黄/右=桃, 補完線は破線）が RViz2 に描画されます。

---

### 3. 実機（実車機体）での運用手順

実機（Jetson / 車載PC）上で実行する際の手順です。

#### A. 機体ハードウェアのみを起動（外部指令待機状態）
センサー（ZED X, IMU, CAN 等）と安全機構を初期化し、速度指令トピック待機状態にします。
```bash
bash bash/1_bringup_hardware.sh
```

#### B. 実機フルシステム（機体 ＋ 自律走行）を一括起動
ハードウェア初期化から、白線検出 (YOLOP)・自己位置推定・周回マップ/QP 走行・信号機検知までの全ノードを 1 コマンドで起動します。
スタート位置 (中央白線の上) に車両を置き、数秒停止させてから (ジャイロバイアス推定) 自動運転を開始してください。1 周目は中央白線の上を走りながら左右境界を記録し、スタート地点に戻ると QP でレーシングラインを作って 2 周目以降それを走ります。
```bash
bash bash/3_bringup_all_nodes.sh
```

#### C. キーボード手動操縦（動作確認・キャリブレーション用）
`twist_mux` 経由で最優先で手動操作を行います。
```bash
bash bash/teleop_keyboard.sh
```

#### D. Mac から WASD で遠隔操作する
実機（Jetson）で `bash/1_bringup_hardware.sh` か `bash/3_bringup_all_nodes.sh` を起動すると、rosbridge WebSocket サーバー（port 9090）が自動で立ち上がります。

> ⚠️ **セキュリティ注意:** この rosbridge は認証なし・全インターフェース待ち受け（0.0.0.0:9090）で自動起動します。信頼できる/隔離されたネットワーク（大会LANなど）以外には機体を接続しないでください。

1. Mac とJetsonを同じLANに接続する。
2. Jetson側でLAN IPを確認する: `hostname -I` （例: `192.168.1.50`）
3. Macのブラウザで `web_simulator/index.html` を開く（`python3 web_simulator/serve.py` などで配信するか、ファイルを直接開く）。
4. 画面上部の「rosbridge URL」欄を `ws://<JetsonのLAN IP>:9090` に書き換えて接続する（デフォルトは `ws://localhost:9090` になっている）。
5. 接続後、WASDキーで操作すると `/aiformula_control/gamepad/cmd_vel` トピック経由で実機の `twist_mux`（gamepad優先度150）に届き、実車が動く。

> ⚠️ **接続断時の挙動:** Mac⇔Jetson間の無線接続が切れて `gamepad` トピックが 0.3 秒以上途絶えると、`twist_mux`（`launchers/sample_launchers/config/twist_mux.yaml`）は自動的に次に優先度の高い入力へフォールバックします。`bringup-all`（自律走行スタック起動）で使用している場合、これは無操作停止ではなく自動運転（`mpc`、優先度50）への切り替わりを意味するため、意図しない挙動に注意してください。

---

## 📚 各機能の詳細ドキュメント

- **自律走行・白線認識・追従制御・信号機検知の詳細**:
  - [src/oit_navigation/README.md](file:///Users/miyanswer/aiformula_machine/src/oit_navigation/README.md)
- **機体アーキテクチャ・トピック仕様・詳細設計**:
  - [docs/reference_ai_formula_oit_2026/](file:///Users/miyanswer/aiformula_machine/docs/reference_ai_formula_oit_2026/)
    - `01_system_architecture.md`: システム全体構成・ノード連携
    - `02_topic_and_interfaces.md`: トピック名・型・インターフェース仕様
    - `03_perception_and_ai.md`: 白線認識・信号機認識アルゴリズム
    - `04_navigation_and_control.md`: 2D BEV 幾何変換・Pure Pursuit 制御理論（※旧方式の資料。現行の走行方式は src/oit_navigation/README.md を参照）
    - `05_launch_and_operations.md`: Launch構成・運用手順
