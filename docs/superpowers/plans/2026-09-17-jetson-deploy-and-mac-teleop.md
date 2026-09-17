# Jetson AGX Orin 実機投入 & Mac WASD 遠隔操作 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Jetson AGX Orin (JetPack 5.1.x / L4T R35, Ubuntu 20.04) 上で `make build` が正しく（x86専用パッケージを掴まずに）動作し、実機を起動できるようにする。あわせて、実機起動時に rosbridge が自動起動し、Mac の `web_simulator` から WASD 操作で実機を動かせるようにする。

**Architecture:** Jetson 検出を `Makefile` に追加し、Jetson専用の `docker/Dockerfile.jetson` + `docker/compose.jetson.yaml` を新設する。ベースイメージは `dustynv/ros:humble-pytorch-l4t-<tag>`（ZEDラッパーの既存Dockerfileと同方式）を使い、ROS 2 Humble のソースビルドと x86 専用 CUDA/TensorRT/PyTorch インストールを回避する。Mac側は `web_simulator` のコード変更は不要（送信先トピックは既に実機の `twist_mux` gamepad 入力と一致）で、実機側の bringup スクリプトに rosbridge の自動起動を追加するだけで配線が繋がる。

**Tech Stack:** Docker / docker compose, GNU Make, ROS 2 Humble (colcon, vcstool, rosdep), bash, dustynv/jetson-containers ベースイメージ。

**Spec:** `docs/superpowers/specs/2026-09-17-jetson-deploy-and-mac-teleop-design.md`

## Global Constraints

- Jetson ベースイメージは `dustynv/ros:humble-pytorch-l4t-${JETSON_BASE_TAG}`（既定値 `r35.3.1`）。x86_64 専用の CUDA toolkit apt インストール・`pip install torch/torchvision/tensorrt-cu12` は Jetson パスでは一切行わない。
- Jetson 検出は `test -f /etc/nv_tegra_release`（`nvidia-smi` は Jetson に存在しないため使わない）。
- `docker/compose.jetson.yaml` は `runtime: nvidia` を使う。既存の x86 GPU機向け `docker/compose.gpu.yaml`（`deploy.resources.reservations.devices` 方式）は変更しない。
- `web_simulator/` 配下のコードは変更しない（デフォルト送信先トピック `/aiformula_control/gamepad/cmd_vel` は既に実機の `twist_mux` gamepad 入力と一致することを確認済み）。
- この会話からJetson実機にSSH等でアクセスできない。検証は (a) Mac (Apple Silicon, arm64) 上でのローカル `docker build`/`docker compose config` によるベストエフォート確認、(b) ユーザーがJetson実機で実行して結果を貼り付ける手動確認、の二本立てで行う。一発で完全動作する保証はなく、ビルドログを見ながらの反復修正を前提とする。
- rviz2 / `rviz_aiformula_plugins` は Jetson イメージには含めない（実車の bringup (`bringup-hw`/`bringup-all`) では RViz は起動されておらず必須ではないため）。Jetson での `make build-ws` は `--packages-skip rviz_aiformula_plugins` を付ける。
- **作業ディレクトリについて:** 以降のタスク中のコマンド例に `cd /Users/miyanswer/aiformula_machine` とあるものは元リポジトリのパスである。実際の作業は本プランが置かれている git worktree のディレクトリ（dispatchする側が指定するカレントディレクトリ）で行うこと。`cd` で元リポジトリに移動しない。

---

## Task 1: Makefile — Jetson検出とJetson用compose配線

**Files:**
- Modify: `Makefile`
- Create: `docker/compose.jetson.yaml`

**Interfaces:**
- Produces: `IS_JETSON` Makefile変数（Jetson上で非空、それ以外で空）、`JETSON_BASE_TAG` Makefile変数（既定 `r35.3.1`、`make build JETSON_BASE_TAG=r35.2.1` で上書き可能）、`docker/compose.jetson.yaml`（後続タスクではこのファイルを直接参照しない。`Makefile` がJetson検出時に自動で読み込む)。

- [ ] **Step 1: `docker/compose.jetson.yaml` を作成する**

```yaml
services:
  aiformula_machine:
    build:
      dockerfile: docker/Dockerfile.jetson
    runtime: nvidia
```

- [ ] **Step 2: `docker compose config` でYAMLの構文・マージ結果を確認する（Mac上で実行可能）**

Run: `docker compose -f compose.yaml -f docker/compose.jetson.yaml config`（現在のworktreeディレクトリで実行）
Expected: エラーなく、マージ後のYAMLが出力され、`dockerfile: docker/Dockerfile.jetson` と `runtime: nvidia` が反映されていること（`docker/Dockerfile.jetson` はこの時点でまだ存在しなくても `config` はYAML検証のみなので成功する）。

- [ ] **Step 3: `Makefile` の GPU/Jetson検出ロジックを書き換える**

`Makefile` の以下のブロック（現在の10〜27行目付近）を置き換える。

現状:
```makefile
OS := $(shell uname -s)
ARCH := $(shell uname -m)
HAS_NVIDIA := $(shell which nvidia-smi 2>/dev/null)

COMPOSE_FILES := -f compose.yaml
ENABLE_CUDA := 0

# Host NVIDIA driver major version (e.g. "535.309.01" -> 535). Empty if no GPU.
NVIDIA_DRIVER_MAJOR := $(shell nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -n1 | cut -d. -f1)

# If running on Linux and NVIDIA GPU is detected, add GPU compose override and enable CUDA
ifeq ($(OS),Linux)
  ifneq ($(HAS_NVIDIA),)
    COMPOSE_FILES += -f docker/compose.gpu.yaml
    ENABLE_CUDA := 1
  endif
endif
```

変更後:
```makefile
OS := $(shell uname -s)
ARCH := $(shell uname -m)
HAS_NVIDIA := $(shell which nvidia-smi 2>/dev/null)
# Jetson (L4T) has no nvidia-smi - /etc/nv_tegra_release exists on every
# L4T/JetPack install instead, so that's the reliable Jetson signal.
IS_JETSON := $(shell test -f /etc/nv_tegra_release && echo 1)
JETSON_BASE_TAG ?= r35.3.1

COMPOSE_FILES := -f compose.yaml
ENABLE_CUDA := 0

# Host NVIDIA driver major version (e.g. "535.309.01" -> 535). Empty if no GPU.
NVIDIA_DRIVER_MAJOR := $(shell nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -n1 | cut -d. -f1)

# If running on Linux and NVIDIA GPU is detected, add GPU compose override and enable CUDA.
# Jetson is checked first since it never has nvidia-smi and needs a
# completely different Dockerfile/base image (see docker/Dockerfile.jetson).
ifeq ($(OS),Linux)
  ifneq ($(IS_JETSON),)
    COMPOSE_FILES += -f docker/compose.jetson.yaml
    ENABLE_CUDA := 1
  else ifneq ($(HAS_NVIDIA),)
    COMPOSE_FILES += -f docker/compose.gpu.yaml
    ENABLE_CUDA := 1
  endif
endif
```

- [ ] **Step 4: x86向け `BUILD_ARGS` とは別に `JETSON_BUILD_ARGS` を定義し、`build`/`rebuild` ターゲットで出し分ける**

`Makefile` の `BUILD_ARGS := ...`（現在の64〜70行目付近）の直後に追加:
```makefile
JETSON_BUILD_ARGS := --build-arg JETSON_BASE_TAG=$(JETSON_BASE_TAG)

# Jetson build args replace (not add to) the x86 CUDA/TensorRT build args
# above - Dockerfile.jetson doesn't declare those ARGs at all (wrong
# architecture packages), so passing them would just be dead/confusing.
ifneq ($(IS_JETSON),)
  FINAL_BUILD_ARGS := $(JETSON_BUILD_ARGS)
else
  FINAL_BUILD_ARGS := $(BUILD_ARGS)
endif
```

`build:` と `rebuild:` ターゲット（現在の150〜154行目）を書き換える:
```makefile
build:
	$(DOCKER_COMPOSE) build $(FINAL_BUILD_ARGS)

rebuild:
	$(DOCKER_COMPOSE) build --no-cache $(FINAL_BUILD_ARGS)
```

- [ ] **Step 5: `help` ターゲットのバナーと確認コマンド案内を更新する**

`help:` ターゲット内の環境表示行（現在の89〜90行目）を書き換える:
```makefile
	@echo "  🖥️  Environment: OS=$(OS) ($(ARCH)) | GPU Mode=$(if $(filter 1,$(ENABLE_CUDA)),$(if $(IS_JETSON),Enabled (Jetson/L4T),Enabled (NVIDIA GPU)),Disabled (CPU/Mac))"
	@echo "  🔥 PyTorch: $(if $(IS_JETSON),bundled in dustynv/ros:humble-pytorch-l4t-$(JETSON_BASE_TAG),$(TORCH_VERSION) / $(TORCH_CUDA_CHANNEL)$(if $(NVIDIA_DRIVER_MAJOR), (host driver $(NVIDIA_DRIVER_MAJOR)),))"
```

`help:` の末尾（`teleop` の行の直後、133行目付近）に、Jetson向けトラブルシュート案内を追加:
```makefile
	@echo ""
	@echo "🤖 [Jetson Troubleshooting]"
	@echo "  cat /etc/nv_tegra_release                 Check installed L4T/JetPack version"
	@echo "  docker info | grep -i runtime              Confirm 'nvidia' runtime is registered"
	@echo "  make bash -> python3 -c \"import torch; print(torch.cuda.is_available())\"  Confirm GPU is visible inside the container (expect True)"
```

- [ ] **Step 6: Mac上でリグレッション確認する**

Run: `make help`（現在のworktreeディレクトリで実行）
Expected: エラーなく表示され、`GPU Mode=Disabled (CPU/Mac)` のまま（Macには `/etc/nv_tegra_release` が存在しないため `IS_JETSON` は空になり、これまで通りの表示になる）。

Run: `make -n build`（現在のworktreeディレクトリで実行）
Expected: `docker compose -f compose.yaml build` が(x86向け`BUILD_ARGS`付きで)表示され、構文エラーが出ないこと。

- [ ] **Step 7: コミットする**

```bash
git add Makefile docker/compose.jetson.yaml
git commit -m "$(cat <<'EOF'
feat: detect Jetson via /etc/nv_tegra_release and add Jetson compose overlay

nvidia-smi doesn't exist on Jetson/L4T, so the existing GPU detection
always fell through to the CPU build there. Route Jetson through a new
docker/compose.jetson.yaml (runtime: nvidia) + docker/Dockerfile.jetson
instead of the x86-only compose.gpu.yaml path.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: `docker/Dockerfile.jetson` — Jetson専用イメージ

**Files:**
- Create: `docker/Dockerfile.jetson`

**Interfaces:**
- Consumes: `docker/entrypoint.sh`（既存、変更不要 — そのままCOPYして流用）
- Produces: イメージがビルドできれば、`ROS_DISTRO=humble` の `/opt/ros/humble/install/setup.bash` と `/opt/extra_ros_ws/install/setup.bash` の2つのオーバーレイが全シェルで自動sourceされる状態（Task 1 の `docker/compose.jetson.yaml` がこの Dockerfile を参照する）。

- [ ] **Step 1: `docker/Dockerfile.jetson` を作成する**

```dockerfile
ARG JETSON_BASE_TAG=r35.3.1
FROM dustynv/ros:humble-pytorch-l4t-${JETSON_BASE_TAG}

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive
# Set explicitly (rather than trusting the base image to have it) since
# every RUN step below that sources /opt/ros/${ROS_DISTRO}/... depends on
# it being correct from the very first RUN.
ENV ROS_DISTRO=humble

# Non-ROS system packages: dev tools + the noVNC/RViz debug stack (kept for
# parity with docker/Dockerfile's PC-standalone verification workflow) +
# CAN utilities. Deliberately NOT installing a CUDA toolkit or PyTorch/
# TensorRT here - the dustynv base image already ships versions matched to
# this exact L4T release; a generic x86_64 wheel/apt repo would either fail
# outright (wrong arch) or silently disable GPU support.
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    curl \
    gdb \
    git \
    htop \
    iputils-ping \
    nano \
    net-tools \
    novnc \
    python3-argcomplete \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    sudo \
    tmux \
    tree \
    vim \
    websockify \
    wget \
    fluxbox \
    xvfb \
    x11vnc \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libasio-dev \
    kmod \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# ROS packages this project needs that the base image doesn't include, built
# the same way sensing/zed-ros2-wrapper/docker/Dockerfile.l4t35_1-humble-release
# vendors ZED's own extra deps: vcs-clone + colcon build into a workspace
# OUTSIDE /aiformula_machine (compose.yaml bind-mounts the whole repo over
# /aiformula_machine at runtime, so anything built there at image-build time
# would just be shadowed and lost).
#
# If a first `make build` fails with "package 'X' not found" during rosdep
# or colcon, add its repo to this block and rebuild - the exact package set
# a given dustynv tag ships is not guaranteed across versions.
# ---------------------------------------------------------------------------
WORKDIR /opt/extra_ros_ws/src
RUN git clone --branch ros2 https://github.com/RobotWebTools/rosbridge_suite.git && \
    git clone --branch humble-devel https://github.com/cra-ros-pkg/robot_localization.git && \
    git clone --branch humble https://github.com/autowarefoundation/ros2_socketcan.git && \
    git clone --branch ros2 https://github.com/ros-drivers/nmea_msgs.git && \
    git clone --branch ros2 https://github.com/ros-perception/vision_msgs.git && \
    git clone --branch humble https://github.com/ros-teleop/teleop_twist_keyboard.git && \
    git clone --branch humble https://github.com/ros-teleop/teleop_twist_joy.git && \
    git clone --branch ros2 https://github.com/ros-drivers/joystick_drivers.git && \
    git clone --branch ros2 https://github.com/ros/joint_state_publisher.git

WORKDIR /opt/extra_ros_ws
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} && \
    rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build --merge-install --parallel-workers \$(nproc) \
    --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Setup noVNC default index
RUN ln -s /usr/share/novnc/vnc.html /usr/share/novnc/index.html 2>/dev/null || true

# Python deps for oit_navigation (mirrors docker/Dockerfile's pip block,
# minus torch/torchvision/onnx-tensorrt which the base image already has
# matched to this L4T release - see the comment above the apt block).
RUN pip3 install --no-cache-dir \
    "setuptools<70.0.0" \
    "numpy<2.0.0" \
    "opencv-python<4.10" \
    python-can \
    pyserial \
    ultralytics \
    yacs \
    scipy \
    tqdm \
    tensorboardX \
    seaborn \
    onnx

# Create non-root user 'rosuser' with sudo privileges
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID $USERNAME 2>/dev/null || true \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash 2>/dev/null || true \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Configure system-wide ROS 2 environment for all shells (interactive,
# non-interactive, and login) - sources the base image's Humble install,
# this image's extra_ros_ws overlay, and (once built) the project's own
# workspace, in that order.
RUN echo "source /opt/ros/${ROS_DISTRO}/install/setup.bash" >> /etc/bash.bashrc \
    && echo "if [ -f /opt/extra_ros_ws/install/setup.bash ]; then source /opt/extra_ros_ws/install/setup.bash; fi" >> /etc/bash.bashrc \
    && echo "if [ -f /aiformula_machine/install/setup.bash ]; then source /aiformula_machine/install/setup.bash; fi" >> /etc/bash.bashrc \
    && mkdir -p /etc/profile.d \
    && echo "source /opt/ros/${ROS_DISTRO}/install/setup.bash" > /etc/profile.d/ros.sh \
    && echo "if [ -f /opt/extra_ros_ws/install/setup.bash ]; then source /opt/extra_ros_ws/install/setup.bash; fi" >> /etc/profile.d/ros.sh \
    && echo "if [ -f /aiformula_machine/install/setup.bash ]; then source /aiformula_machine/install/setup.bash; fi" >> /etc/profile.d/ros.sh \
    && chmod +x /etc/profile.d/ros.sh

# Configure .bashrc for rosuser (prepend to ensure it runs even for non-interactive subshells)
RUN sed -i '1i source /opt/ros/'"${ROS_DISTRO}"'/install/setup.bash' /home/$USERNAME/.bashrc \
    && echo "if [ -f /opt/extra_ros_ws/install/setup.bash ]; then source /opt/extra_ros_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /aiformula_machine/install/setup.bash ]; then source /aiformula_machine/install/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "eval \"\$(register-python-argcomplete3 ros2)\"" >> /home/$USERNAME/.bashrc \
    && echo "eval \"\$(register-python-argcomplete3 colcon)\"" >> /home/$USERNAME/.bashrc \
    && echo "export ROS_DOMAIN_ID=100" >> /home/$USERNAME/.bashrc

ENV DISPLAY=:1
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV QT_X11_NO_MITSHM=1

# Set up entrypoint (same as docker/Dockerfile - starts Xvfb/fluxbox/x11vnc/websockify)
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Workspace directory
WORKDIR /aiformula_machine
RUN chown -R $USERNAME:$USERNAME /aiformula_machine 2>/dev/null || true

USER $USERNAME

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

- [ ] **Step 2: Mac (Apple Silicon, arm64) 上でベストエフォートのビルド検証を行う**

Jetsonにこのセッションから直接アクセスできないため、これが実行可能な唯一のローカル検証。Macはarm64なので `dustynv/ros:humble-pytorch-l4t-*` イメージ自体はpull・実行できる(GPU関連の初期化は当然スキップ/失敗しうるが、apt/rosdep/colconのビルド手順の妥当性は検証できる)。

Run（現在のworktreeディレクトリで実行）: `docker build -f docker/Dockerfile.jetson -t aiformula_machine:jetson-test --build-arg JETSON_BASE_TAG=r35.3.1 .`
Expected: 全レイヤーが成功する。失敗した場合はエラーになった `RUN` ステップ（特に `git clone` のブランチ名や `rosdep install`/`colcon build` のパッケージ不足）を特定し、Dockerfile内の該当ブロックを修正して再実行する。イメージが数GB規模になり時間がかかる点は許容する。

もしMac上でのビルドがネットワーク/pull容量などの理由で現実的でない場合は、この検証をスキップし、Task 4での実機検証に委ねる（その場合はこのステップの結果を「Mac未検証」として次のステップ/引き継ぎメモに明記する）。

- [ ] **Step 3: コミットする**

```bash
git add docker/Dockerfile.jetson
git commit -m "$(cat <<'EOF'
feat: add Jetson-specific Dockerfile based on dustynv/ros humble-pytorch

JetPack 5.1.x's Ubuntu 20.04 userspace has no official Humble apt
packages, and sourcing ROS 2 Humble from scratch takes hours. Base on
dustynv/ros:humble-pytorch-l4t-<tag> instead (same approach already
used by the vendored sensing/zed-ros2-wrapper Jetson Dockerfile), which
ships Humble plus a matched CUDA/cuDNN/TensorRT/PyTorch for the host's
exact L4T release. Vendor the handful of ROS packages the base image
doesn't include (rosbridge_suite, robot_localization, ros2_socketcan,
etc.) into a separate /opt/extra_ros_ws overlay built at image time.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: 実機起動時の rosbridge 自動起動 + Mac操作手順のREADME追記

**Files:**
- Modify: `bash/1_bringup_hardware.sh`
- Modify: `bash/3_bringup_all_nodes.sh`
- Modify: `README.md`

**Interfaces:**
- Consumes: Task 2 で `/opt/extra_ros_ws` に colcon build 済みの `rosbridge_server` パッケージ（`ros2 launch rosbridge_server rosbridge_websocket_launch.xml` が使える状態）。
- Produces: 実機起動時、port 9090 で rosbridge WebSocket が自動起動している状態（`compose.yaml` が既に `9090:9090` を公開しているため、追加のネットワーク変更は不要)。

- [ ] **Step 1: `bash/1_bringup_hardware.sh` に rosbridge 自動起動を追加する**

`bash/1_bringup_hardware.sh` の `echo "  [AI Formula] Launching Hardware Bringup..."` の直前(現在の末尾, `ros2 launch sample_launchers hardware_bringup.launch.py` の手前)に追加:

```bash
# rosbridge WebSocket (port 9090) - lets the Mac web_simulator drive this
# vehicle over the LAN via WASD (see web_simulator/index.html's rosbridge
# URL field). Idempotent so re-running this script doesn't double-launch it.
start_rosbridge() {
    if ! pgrep -f "rosbridge_websocket" > /dev/null; then
        echo "[INFO] Starting rosbridge_server (port 9090) for remote (Mac) teleop..."
        ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/rosbridge.log 2>&1 &
        sleep 1
    fi
}

start_rosbridge
```

- [ ] **Step 2: `bash/3_bringup_all_nodes.sh` にも同じ関数を追加する**

`bash/1_bringup_hardware.sh` と全く同じ `start_rosbridge()` 関数定義と呼び出しを、`bash/3_bringup_all_nodes.sh` の `echo "  [AI Formula] Launching Full System..."` の直前(`ros2 launch sample_launchers all_system_2027.launch.py ...` の手前)に追加する。

- [ ] **Step 3: 構文チェック(Mac上で実行可能)**

Run: `bash -n bash/1_bringup_hardware.sh && bash -n bash/3_bringup_all_nodes.sh && echo OK`（現在のworktreeディレクトリで実行）
Expected: `OK` とだけ出力される（構文エラーなし）。

- [ ] **Step 4: README に Mac操作手順を追記する**

`README.md` の `### 3. 実機（実車機体）での運用手順` セクション（C. キーボード手動操縦の直後、129行目付近）に新しい項を追加:

```markdown
#### D. Mac から WASD で遠隔操作する
実機（Jetson）で `bash/1_bringup_hardware.sh` か `bash/3_bringup_all_nodes.sh` を起動すると、rosbridge WebSocket サーバー（port 9090）が自動で立ち上がります。

1. Mac とJetsonを同じLANに接続する。
2. Jetson側でLAN IPを確認する: `hostname -I` （例: `192.168.1.50`）
3. Macのブラウザで `web_simulator/index.html` を開く（`python3 web_simulator/serve.py` などで配信するか、ファイルを直接開く）。
4. 画面上部の「rosbridge URL」欄を `ws://<JetsonのLAN IP>:9090` に書き換えて接続する（デフォルトは `ws://localhost:9090` になっている）。
5. 接続後、WASDキーで操作すると `/aiformula_control/gamepad/cmd_vel` トピック経由で実機の `twist_mux`（gamepad優先度150）に届き、実車が動く。
```

- [ ] **Step 5: コミットする**

```bash
git add bash/1_bringup_hardware.sh bash/3_bringup_all_nodes.sh README.md
git commit -m "$(cat <<'EOF'
feat: auto-start rosbridge on real-vehicle bringup for Mac WASD teleop

web_simulator already targets /aiformula_control/gamepad/cmd_vel by
default, which matches twist_mux's real gamepad input topic - the only
missing piece was rosbridge actually running on the vehicle. Start it
from both bringup scripts instead of requiring a manual `make
rosbridge`, and document the Mac-side connection steps.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Jetsonセットアップ手順 + TensorRT自動コンパイルの挙動をREADMEに明記

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: なし（ドキュメントのみ）。

- [ ] **Step 1: `src/oit_navigation/oit_navigation/yolop_lane_detector.py:290-349` の既存の自動コンパイル挙動を確認する**

Run: `sed -n '290,349p' src/oit_navigation/oit_navigation/yolop_lane_detector.py`
Expected: `_init_tensorrt_detector` が、現在のGPU向けのタグ付きengineファイル（例: `..._sm87.engine`）が無ければ `export_engine_for_current_gpu` で自動ビルドし、失敗時はPyTorch推論にフォールバックするコードが確認できる（`models/*_sm75.engine` はRTX2070向けでOrinでは使われず、初回起動時に自動で作り直される）。

- [ ] **Step 2: README にJetsonセットアップ手順のセクションを追加する**

`README.md` の `### 1. Docker 環境のセットアップ` セクション内、GPU自動判定の説明ブロック（68〜71行目付近）の直後に追加:

```markdown
> **🤖 Jetson AGX Orin (JetPack 5.1.x / L4T R35) での実行:**
> - `make build` 実行前に `cat /etc/nv_tegra_release` で搭載中のL4Tバージョンを確認してください。`docker/Dockerfile.jetson` は既定で `r35.3.1`（JetPack 5.1.1相当）のベースイメージを使いますが、異なる場合は `make build JETSON_BASE_TAG=r35.2.1` のように上書きしてください（ズレると `torch.cuda.is_available()` が `False` になります）。
> - `docker info | grep -i runtime` で `nvidia` ランタイムが登録されていることを事前に確認してください（JetPack標準セットアップ済みであれば通常は有効です）。
> - `models/*_rtx_2070_..._sm75.engine` はRTX2070(sm75)向けのTensorRTエンジンで、Orin(sm87)では使われません。`yolop_lane_detector` は起動時に現在のGPU向けのエンジンが無ければ自動でコンパイルし直すため（`src/oit_navigation/oit_navigation/yolop_lane_detector.py` の `_init_tensorrt_detector` 参照）、追加の手動作業は不要ですが、初回起動時は数分ほど余分に時間がかかります。
> - Jetsonでは `rviz_aiformula_plugins` パッケージ（RViz専用プラグイン、実車走行には不要）はビルド対象から外しています: `make build-ws` の代わりに `docker compose exec aiformula_machine bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-skip rviz_aiformula_plugins"` を使ってください。
```

- [ ] **Step 3: コミットする**

```bash
git add README.md
git commit -m "$(cat <<'EOF'
docs: document Jetson setup steps and TensorRT auto-recompile behavior

Explains the JETSON_BASE_TAG override, the nvidia runtime prerequisite,
and that a mismatched-architecture TensorRT engine self-heals on first
run instead of needing a manual re-export step.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```
