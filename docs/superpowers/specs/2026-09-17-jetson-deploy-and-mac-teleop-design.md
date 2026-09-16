# Jetson AGX Orin 実機投入 & Mac からの WASD 遠隔操作 設計書

- 日付: 2026-09-17
- 対象機体: Jetson AGX Orin 64GB (hostname: aiformula-oit, JetPack 5.1.x / L4T R35 / Ubuntu 20.04)
- 操作端末: Mac (Apple Silicon/Intel) の `web_simulator` から WASD 操作

## 背景・現状の問題

現行の `docker/Dockerfile` / `Makefile` は x86_64 の NVIDIA GPU デスクトップ機を前提にしており、Jetson AGX Orin では以下の理由でそのまま動作しない、または致命的に劣化する。

1. `Makefile` の GPU 検知が `nvidia-smi` の有無に依存している。Jetson には `nvidia-smi` が存在しないため（`tegrastats` 系のみ）、常に `ENABLE_CUDA=0` と誤判定され、CPU版 PyTorch がビルドされてOrinのGPU性能が使われない。
2. `ENABLE_CUDA=1` 相当の分岐は `cuda-keyring_...ubuntu2204/x86_64` や `torch --index-url .../cu121`、`tensorrt-cu12` など **x86_64 専用パッケージ**を掴みに行く実装であり、Jetson (aarch64) 上では成立しない。
3. ROS 2 Humble の公式バイナリ(apt)は Ubuntu 22.04(Jammy) 向けのみで、JetPack 5.1.x のユーザ空間である Ubuntu 20.04(Focal) には存在しない。Focal 上で Humble をソースビルドすると初回ビルドが数時間規模になり、「ビルド時間が長すぎる」という要望に反する。
4. `models/*_rtx_2070_..._sm75.engine` は RTX 2070 (Turing, sm75) 向けにビルドされた TensorRT エンジンであり、Orin (Ampere, sm87) では読み込めない。オンデバイスでの再エクスポートが必要。
5. Mac 側 `web_simulator` から実機を操作する経路（rosbridge 経由）は配線自体はほぼ揃っているが、実機側で rosbridge を自動起動する導線がなく、手動 `make rosbridge` が必要。

## 方針

### A. Jetson 専用 Dockerfile (`docker/Dockerfile.jetson`)

- ベースイメージ: `dustynv/ros:humble-pytorch-l4t-r35.3.1`（Docker Hub 上に実在確認済み、linux/arm64）。
  - 同梱リポジトリ `sensing/zed-ros2-wrapper/docker/Dockerfile.l4t35_1-humble-release` が同種の `dustynv/ros:humble-ros-base-l4t-r35.1.0` を採用しており、本プロジェクトの ZED 運用と方式を揃える。
  - このイメージは L4T のバージョンに正しく紐付いた CUDA / cuDNN / TensorRT / PyTorch を同梱しているため、**x86_64 用の CUDA toolkit apt インストールや pip 経由の torch/tensorrt インストールは一切行わない**（現行 Dockerfile のそれらの分岐は Jetson パスでは使わず削除する）。
  - L4T のマイナーバージョンは機体ごとに異なりうるため、ビルド引数 `JETSON_BASE_TAG`（既定値 `r35.3.1`）で選べるようにする。ユーザには `cat /etc/nv_tegra_release` で確認し、必要なら上書きしてもらう。ズレると `torch.cuda.is_available() == False` になる点をREADMEに明記する。
- ROS パッケージのうち、ベースイメージに含まれない可能性が高いもの（`rosbridge_suite`, `robot_localization`, `ros2_socketcan`(can_msgs含む), `teleop_twist_keyboard`, `teleop_twist_joy`, `joy`, `vision_msgs`, `diagnostic_updater`, 必要なら `rviz2`/`xacro`/`joint_state_publisher(-gui)`）は、ZED ラッパーの Dockerfile と同じ手法で `/opt/extra_ros_ws/src` に `git clone` し、`rosdep install --from-paths --ignore-src` → `colcon build` する。
  - このワークスペースは **bind mount 対象外**（`/aiformula_machine` の外）に置き、image layer として永続させる。これにより `.:/aiformula_machine` の bind mount で上書きされず、かつ `make build-ws`（自前パッケージのビルド）とは独立してキャッシュされる。
  - 実際に何が不足しているかは `rosdep install` の出力で正確に判明するため、事前リストは「候補」であり、初回ビルドでの調整を前提とする（後述「検証・反復方針」）。
- noVNC/x11vnc/fluxbox/xvfb 等の GUI デバッグ環境は README の検証ワークフロー(RViz2)に必須のため維持するが、`ros-humble-desktop` 相当の不要なデモ・チュートリアルパッケージはビルド対象に含めない。
- Python 側は既存の `numpy<2.0.0` / `opencv-python<4.10` / `ultralytics` 等のpipインストールのみ踏襲し、torch/torchvision/tensorrt 関連の pip インストールは行わない（ベースイメージのものを使う）。

### B. Makefile / compose の変更

- Jetson 検出を `nvidia-smi` ではなく `test -f /etc/nv_tegra_release` に変更する（全 L4T 機に存在するファイル）。
- 検出時は `docker/compose.jetson.yaml` を追加読み込みする新しい分岐を作る。既存の x86 GPU 機向け `docker/compose.gpu.yaml`（`deploy.resources.reservations.devices` 方式）とは別経路にし、既存の x86 デスクトップGPU動作には影響を与えない。
- `docker/compose.jetson.yaml`:
  - `build.dockerfile: docker/Dockerfile.jetson` を指定。
  - `runtime: nvidia` を指定（Jetson は `nvidia-container-runtime` の csv マウント方式で GPU デバイス/ライブラリをコンテナに渡すため）。前提として `docker info` で `nvidia` runtime が登録されていることをREADMEに前提条件として明記する。
- x86 向け `BUILD_ARGS`（`TORCH_CUDA_CHANNEL` 等）は Jetson ビルドでは使わない値なので、Jetson 検出時は渡さない（無害だが混乱を避けるため）。

### C. TensorRT エンジン再生成

- `models/*_sm75.engine` は Orin では使えないため、Jetson 実機上で `ros2 run oit_navigation export_tensorrt`（または直接 `python3 export_tensorrt.py`）を実行し、Orin 向けエンジンを再生成する手順を README / セットアップ手順に追記する。
- 既存コードは TensorRT 読み込み失敗時に PyTorch 推論へフォールバックする実装（Dockerfile コメントに記載あり）になっているため、再生成前でも致命的クラッシュにはならない想定。ただし性能低下するため、Jetson セットアップの必須手順として明記する。

### D. Mac → 実機の WASD 操作

- 調査の結果、`web_simulator/index.html` のデフォルト送信先トピック `/aiformula_control/gamepad/cmd_vel` は実機 `twist_mux` の `gamepad` 入力（優先度150、`launchers/sample_launchers/config/twist_mux.yaml` / `topic_list.yaml` 参照）と**既に一致**しており、CLAUDE.md が求めるシミュレータ⇔実機の整合性チェックの結果、今回はズレなし・変更不要。
- 実機側で `rosbridge_server` (port 9090) が起動しているだけで、Mac のブラウザから `ws://<JetsonのLAN IP>:9090` を指定すれば WASD 操作が届く配線になっている。`compose.yaml` は既に `9090:9090` を公開しているため追加のネットワーク設定は不要。
- 変更点: `bash/1_bringup_hardware.sh`（もしくは新規 launch）に rosbridge の起動を組み込み、実機起動時に自動で WebSocket が立ち上がるようにする。現行の `make rosbridge` (手動、かつ apt install フォールバック付き) は Jetson では Focal 上に apt debパッケージが無いため使えないので、上記 C. の overlay workspace 側でビルド済みの rosbridge を使う形に統一する。
- README に「Mac側の操作手順」（ブラウザで `web_simulator/index.html` を開き、rosbridge URL に Jetson の LAN IP を入力して接続）を追記する。

## 検証・反復方針（重要な制約）

この会話からは Jetson 実機に SSH 等で直接アクセスできないため、Dockerfile.jetson は**一発で完全に動く保証ができない**。特に「ベースイメージに何が含まれ何が含まれないか」は実際に `docker build` してみないと確定しない部分がある。

- 初回実装は「最有力の構成」で用意するが、ユーザーに Jetson 実機で `make build` を実行してもらい、エラー内容（特に `rosdep install` や `colcon build` の失敗ログ）を貼り付けてもらいながら反復修正することを前提とする。
- `Makefile` の `help` に、Jetson 上での典型的な確認コマンド（`cat /etc/nv_tegra_release`、`docker info | grep -i runtime`、コンテナ内 `python3 -c "import torch;print(torch.cuda.is_available())"`）を追記し、切り分けをしやすくする。

## スコープ外（今回は対応しない）

- JetPack 6 への再フラッシュ（ユーザーが明示的に見送りを選択）。
- dusty_nv 以外の Isaac ROS 系イメージの採用（現時点で Humble/Focal 向けの Isaac apt リポジトリが障害中であることを確認済みのため見送り）。
- `web_simulator` 側のコード変更（トピック名が既に一致しているため不要と確認）。
