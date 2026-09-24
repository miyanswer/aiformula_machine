# ==============================================================================
# aiformula_machine Docker Management Makefile
# ==============================================================================

SERVICE_NAME := aiformula_machine
CONTAINER_NAME := aiformula_machine_humble

# ------------------------------------------------------------------------------
# Environment & GPU Auto-Detection (Mac vs Ubuntu/Linux)
# ------------------------------------------------------------------------------
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

# Pick a PyTorch CUDA build the *host driver* can actually run.
# A wheel newer than the driver loads but fails at runtime with
# "The NVIDIA driver on your system is too old" -> torch.cuda.is_available() == False.
#   driver >= 530 : CUDA 12.1 OK -> cu121
#   driver >= 520 : CUDA 11.8 only -> cu118
#   driver <  520 : too old, fall back to CPU build
ifeq ($(ENABLE_CUDA),1)
  ifeq ($(IS_JETSON),)
    DRIVER_OK_121 := $(shell [ "$(NVIDIA_DRIVER_MAJOR)" -ge 530 ] 2>/dev/null && echo 1)
    DRIVER_OK_118 := $(shell [ "$(NVIDIA_DRIVER_MAJOR)" -ge 520 ] 2>/dev/null && echo 1)
    ifeq ($(DRIVER_OK_121),1)
      TORCH_CUDA_CHANNEL := cu121
      TORCH_VERSION := 2.5.1
      TORCHVISION_VERSION := 0.20.1
      CUDA_APT_VERSION := 12-1
      TENSORRT_CU := cu12
    else ifeq ($(DRIVER_OK_118),1)
      TORCH_CUDA_CHANNEL := cu118
      TORCH_VERSION := 2.6.0
      TORCHVISION_VERSION := 0.21.0
      CUDA_APT_VERSION := 11-8
      TENSORRT_CU := cu11
    else
      ENABLE_CUDA := 0
    endif
  endif
endif

# CPU fallback pins (also used when the driver is too old for any CUDA wheel)
TORCH_CUDA_CHANNEL ?= cpu
TORCH_VERSION ?= 2.5.1
TORCHVISION_VERSION ?= 0.20.1
CUDA_APT_VERSION ?= 12-1
TENSORRT_CU ?= cu12
# TensorRT's own PyPI release cadence, not tied to the CUDA/torch channel above.
TENSORRT_VERSION ?= 10.8.0.43

BUILD_ARGS := --build-arg ENABLE_CUDA=$(ENABLE_CUDA) \
              --build-arg TORCH_CUDA_CHANNEL=$(TORCH_CUDA_CHANNEL) \
              --build-arg TORCH_VERSION=$(TORCH_VERSION) \
              --build-arg TORCHVISION_VERSION=$(TORCHVISION_VERSION) \
              --build-arg CUDA_APT_VERSION=$(CUDA_APT_VERSION) \
              --build-arg TENSORRT_CU=$(TENSORRT_CU) \
              --build-arg TENSORRT_VERSION=$(TENSORRT_VERSION)

# On Jetson, rviz_aiformula_plugins (RViz-only plugins, not needed to drive
# the real vehicle) can't build - the headless Jetson image intentionally
# doesn't ship rviz2/rviz_common (see docker/Dockerfile.jetson). Skip it
# automatically so `make build-ws` / `make build-pkg` just work on Jetson
# without users needing to remember a manual --packages-skip flag.
COLCON_SKIP := $(if $(IS_JETSON),--packages-skip rviz_aiformula_plugins,)

JETSON_BUILD_ARGS := --build-arg JETSON_BASE_TAG=$(JETSON_BASE_TAG)

# Jetson build args replace (not add to) the x86 CUDA/TensorRT build args
# above - Dockerfile.jetson doesn't declare those ARGs at all (wrong
# architecture packages), so passing them would just be dead/confusing.
ifneq ($(IS_JETSON),)
  FINAL_BUILD_ARGS := $(JETSON_BUILD_ARGS)
else
  FINAL_BUILD_ARGS := $(BUILD_ARGS)
endif

DOCKER_COMPOSE := docker compose $(COMPOSE_FILES)

# Parameters with defaults
DEVICE ?= $(if $(filter 1,$(ENABLE_CUDA)),cuda,cpu)
VIDEO ?=
BACKEND ?= yolop
PKG ?=

.PHONY: help up down stop restart build rebuild ps logs bash shell root-bash root \
        build-ws colcon clean test-pc test test-tl test-lane test-yolop test-ufld \
        verification-gui vgui open-rviz gui open-vgui stop-nodes kill \
        rosbridge sim open-sim sim-nav rqt rqt-graph \
        bringup-hw bringup-all teleop zed-check

# Default: Show help message
help:
	@echo "========================================================================"
	@echo "  🏎️  AI Formula Machine - Docker & Development Commands"
	@echo "  🖥️  Environment: OS=$(OS) ($(ARCH)) | GPU Mode=$(if $(filter 1,$(ENABLE_CUDA)),$(if $(IS_JETSON),Enabled (Jetson/L4T),Enabled (NVIDIA GPU)),Disabled (CPU/Mac))"
	@echo "  🔥 PyTorch: $(if $(IS_JETSON),bundled in dustynv/ros:humble-pytorch-l4t-$(JETSON_BASE_TAG),$(TORCH_VERSION) / $(TORCH_CUDA_CHANNEL)$(if $(NVIDIA_DRIVER_MAJOR), (host driver $(NVIDIA_DRIVER_MAJOR)),))"
	@echo "========================================================================"
	@echo ""
	@echo "📦 [Container Management]"
	@echo "  make up               Start Docker container in background"
	@echo "  make down             Stop and remove Docker containers"
	@echo "  make stop             Stop running container"
	@echo "  make restart          Restart Docker container"
	@echo "  make build            Build Docker image (Auto-detects CPU/CUDA)"
	@echo "  make rebuild          Rebuild Docker image without cache"
	@echo "  make ps               Check container status"
	@echo "  make logs             Show container logs"
	@echo ""
	@echo "💻 [Shell Access]"
	@echo "  make bash             Open interactive bash shell as 'rosuser'"
	@echo "  make root             Open interactive bash shell as 'root'"
	@echo ""
	@echo "🔨 [Build & Clean]"
	@echo "  make build-ws         Build all packages (colcon build --symlink-install)"
	@echo "  make build-pkg PKG=xx Build specific package (e.g. make build-pkg PKG=oit_navigation)"
	@echo "  make clean            Remove build/, install/, and log/ directories"
	@echo ""
	@echo "🧪 [PC Standalone Video Test]"
	@echo "  make test-pc          Run video test (lane detection + Traffic Light + RViz)"
	@echo "                        Options: DEVICE=cpu|cuda|mps  VIDEO=/path/to/video.mp4  BACKEND=yolop|ufld"
	@echo "  make test-tl          Test traffic light detection & distance estimation"
	@echo "  make test-lane        Test lane detection only (left/center/right, BACKEND=yolop|ufld)"
	@echo "  make test-yolop       = make test-lane BACKEND=yolop"
	@echo "  make test-ufld        = make test-lane BACKEND=ufld (needs models/ufld_honda_finetuned_best.pth)"
	@echo "  make vgui             Run Web Verification GUI (open http://localhost:8090)"
	@echo "  make stop-nodes       Kill all running ROS 2 nodes inside container"
	@echo ""
	@echo "🌐 [Web Simulator & UIs]"
	@echo "  make rosbridge        Start rosbridge WebSocket server on port 9090"
	@echo "  make open-sim (sim)   Open 3D Web Simulator in browser (http://localhost:8000)"
	@echo "  make sim-nav          Run lane_detector + odom_imu_localizer + lane_navigator against the"
	@echo "                        Web Simulator (its 'ROS2連携' mode, needs 'make rosbridge')  BACKEND=yolop|ufld"
	@echo "  make open-rviz (gui)  Open RViz2 Web Display in browser (http://localhost:8080)"
	@echo "  make rqt-graph        Open rqt_graph in browser GUI (http://localhost:8080)"
	@echo "  make rqt              Open full rqt dashboard in browser GUI (http://localhost:8080)"
	@echo "  make open-vgui        Open Web Verification GUI in browser (http://localhost:8090)"
	@echo ""
	@echo "🏎️ [Real Vehicle Operations]"
	@echo "  make bringup-hw       Launch hardware nodes only"
	@echo "  make bringup-all      Launch hardware + full autonomous stack"
	@echo "  make teleop           Run keyboard teleoperation"
	@echo "  make zed-check        Check ZED SDK / argus socket / can0 / IMU visibility in container"
	@echo ""
	@echo "🤖 [Jetson Troubleshooting]"
	@echo "  cat /etc/nv_tegra_release                 Check installed L4T/JetPack version"
	@echo "  docker info | grep -i runtime              Confirm 'nvidia' runtime is registered"
	@echo "  make bash -> python3 -c \"import torch; print(torch.cuda.is_available())\"  Confirm GPU is visible inside the container (expect True)"
	@echo "========================================================================"

# ------------------------------------------------------------------------------
# Container Management
# ------------------------------------------------------------------------------

up:
	$(DOCKER_COMPOSE) up -d

down:
	$(DOCKER_COMPOSE) down

stop:
	$(DOCKER_COMPOSE) stop

restart: down up

build:
	$(DOCKER_COMPOSE) build $(FINAL_BUILD_ARGS)

rebuild:
	$(DOCKER_COMPOSE) build --no-cache $(FINAL_BUILD_ARGS)

ps:
	$(DOCKER_COMPOSE) ps

logs:
	$(DOCKER_COMPOSE) logs -f $(SERVICE_NAME)

# ------------------------------------------------------------------------------
# Shell Access (Auto-starts container if not running)
# ------------------------------------------------------------------------------

bash shell exec:
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		echo "[INFO] Container is not running. Starting $(SERVICE_NAME)..."; \
		$(DOCKER_COMPOSE) up -d; \
	fi
	$(DOCKER_COMPOSE) exec -it $(SERVICE_NAME) bash

root root-bash:
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		echo "[INFO] Container is not running. Starting $(SERVICE_NAME)..."; \
		$(DOCKER_COMPOSE) up -d; \
	fi
	$(DOCKER_COMPOSE) exec -it -u root $(SERVICE_NAME) bash

# ------------------------------------------------------------------------------
# Build & Workspace Management
# ------------------------------------------------------------------------------

build-ws colcon:
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		$(DOCKER_COMPOSE) up -d; \
	fi
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install $(COLCON_SKIP)"

build-pkg:
	@if [ -z "$(PKG)" ]; then \
		echo "[ERROR] Please specify PKG. Example: make build-pkg PKG=oit_navigation"; \
		exit 1; \
	fi
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		$(DOCKER_COMPOSE) up -d; \
	fi
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select $(PKG) --symlink-install $(COLCON_SKIP)"

clean:
	rm -rf build install log

# ------------------------------------------------------------------------------
# Standalone Video Testing (PC Verification)
# ------------------------------------------------------------------------------

test-pc test:
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		$(DOCKER_COMPOSE) up -d; \
	fi
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c \
		"source /opt/ros/humble/setup.bash && source install/setup.bash && \
		 ros2 launch oit_navigation video_test.launch.py \
		 $(if $(VIDEO),video_path:=$(VIDEO),) \
		 use_device:=$(DEVICE) \
		 backend:=$(BACKEND) \
		 rviz:=true"

test-tl:
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		$(DOCKER_COMPOSE) up -d; \
	fi
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c \
		"source /opt/ros/humble/setup.bash && source install/setup.bash && \
		 ros2 launch oit_navigation traffic_light_video_test.launch.py \
		 $(if $(VIDEO),video_path:=$(VIDEO),) \
		 device:=$(DEVICE)"

test-lane:
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		$(DOCKER_COMPOSE) up -d; \
	fi
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c \
		"source /opt/ros/humble/setup.bash && source install/setup.bash && \
		 ros2 launch oit_navigation video_test.launch.py \
		 $(if $(VIDEO),video_path:=$(VIDEO),) \
		 use_device:=$(DEVICE) \
		 backend:=$(BACKEND) \
		 traffic_light:=false \
		 rviz:=true"

test-yolop:
	$(MAKE) test-lane BACKEND=yolop

test-ufld:
	$(MAKE) test-lane BACKEND=ufld

vgui verification-gui:
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		$(DOCKER_COMPOSE) up -d; \
	fi
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c \
		"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run oit_navigation verification_gui"

stop-nodes kill:
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c \
		"pkill -9 -f 'ros2|rviz2|video_publisher|lane_detector|lane_navigator|odom_imu_localizer|traffic_light|robot_state_publisher|joint_state_publisher' || true"

# ------------------------------------------------------------------------------
# Web GUI Launchers (Host browser)
# ------------------------------------------------------------------------------

open-rviz gui:
	@which open > /dev/null && open http://localhost:8080 || which xdg-open > /dev/null && xdg-open http://localhost:8080 || echo "Open http://localhost:8080 in your browser"

open-vgui:
	@which open > /dev/null && open http://localhost:8090 || which xdg-open > /dev/null && xdg-open http://localhost:8090 || echo "Open http://localhost:8090 in your browser"

open-sim sim:
	@which open > /dev/null && open http://localhost:8000 || which xdg-open > /dev/null && xdg-open http://localhost:8000 || echo "Open http://localhost:8000 in your browser"

rqt-graph:
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		$(DOCKER_COMPOSE) up -d; \
	fi
	@which open > /dev/null && open http://localhost:8080 || which xdg-open > /dev/null && xdg-open http://localhost:8080 || echo "Open http://localhost:8080 in your browser"
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c "source /opt/ros/humble/setup.bash && rqt_graph"

rqt:
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		$(DOCKER_COMPOSE) up -d; \
	fi
	@which open > /dev/null && open http://localhost:8080 || which xdg-open > /dev/null && xdg-open http://localhost:8080 || echo "Open http://localhost:8080 in your browser"
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c "source /opt/ros/humble/setup.bash && rqt"

rosbridge:
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		$(DOCKER_COMPOSE) up -d; \
	fi
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c \
		"source /opt/ros/humble/setup.bash && \
		 if ! ros2 pkg list | grep -q '^rosbridge_server$$'; then \
		   echo '[INFO] Installing ros-humble-rosbridge-server...'; \
		   sudo apt-get update && sudo apt-get install -y ros-humble-rosbridge-server; \
		 fi && \
		 ros2 launch rosbridge_server rosbridge_websocket_launch.xml"

sim-nav:
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		$(DOCKER_COMPOSE) up -d; \
	fi
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c \
		"source /opt/ros/humble/setup.bash && source install/setup.bash && \
		 ros2 launch oit_navigation simulator_test.launch.py \
		 use_device:=$(DEVICE) \
		 backend:=$(BACKEND)"

# ------------------------------------------------------------------------------
# Real Vehicle Operations
# ------------------------------------------------------------------------------

# 実機ホスト(Jetson)は ROS 2 Foxy / Humble 未導入のため、bringup はコンテナ内で実行する
# (ZED SDK・extra_ros_ws・ビルド済み install/ は全てコンテナ側にある)。
define ENSURE_UP
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		$(DOCKER_COMPOSE) up -d; \
	fi
endef

bringup-hw:
	$(ENSURE_UP)
	$(DOCKER_COMPOSE) exec -it $(SERVICE_NAME) bash bash/1_bringup_hardware.sh

bringup-all:
	$(ENSURE_UP)
	$(DOCKER_COMPOSE) exec -it $(SERVICE_NAME) bash bash/3_bringup_all_nodes.sh

teleop:
	$(ENSURE_UP)
	$(DOCKER_COMPOSE) exec -it $(SERVICE_NAME) bash bash/teleop_keyboard.sh

# ZED X がコンテナ内から見えるかの確認 (ホスト側の zed_x_daemon / nvargus-daemon 前提)
zed-check:
	$(ENSURE_UP)
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c \
		"ls -l /usr/local/zed/lib/libsl_zed.so* && \
		 grep -h "set(PACKAGE_VERSION " /usr/local/zed/zed-config-version.cmake; \
		 test -S /tmp/argus_socket && echo '[OK] /tmp/argus_socket' || echo '[NG] /tmp/argus_socket missing (host: sudo systemctl restart nvargus-daemon zed_x_daemon)'; \
		 ip link show can0 >/dev/null 2>&1 && echo '[OK] can0 visible' || echo '[NG] can0 not visible'; \
		 ls /dev/ttyUSB0 >/dev/null 2>&1 && echo '[OK] /dev/ttyUSB0' || echo '[NG] /dev/ttyUSB0 missing'"
