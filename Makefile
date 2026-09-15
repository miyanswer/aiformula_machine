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

COMPOSE_FILES := -f compose.yaml
ENABLE_CUDA := 0

# If running on Linux and NVIDIA GPU is detected, add GPU compose override and enable CUDA
ifeq ($(OS),Linux)
  ifneq ($(HAS_NVIDIA),)
    COMPOSE_FILES += -f docker/compose.gpu.yaml
    ENABLE_CUDA := 1
  endif
endif

DOCKER_COMPOSE := docker compose $(COMPOSE_FILES)

# Parameters with defaults
DEVICE ?= $(if $(filter 1,$(ENABLE_CUDA)),cuda,cpu)
VIDEO ?=
PKG ?=

.PHONY: help up down stop restart build rebuild ps logs bash shell root-bash root \
        build-ws colcon clean test-pc test test-tl test-yolop test-control \
        verification-gui vgui open-rviz gui open-vgui stop-nodes kill \
        rosbridge sim open-sim rqt rqt-graph \
        bringup-hw bringup-all teleop

# Default: Show help message
help:
	@echo "========================================================================"
	@echo "  🏎️  AI Formula Machine - Docker & Development Commands"
	@echo "  🖥️  Environment: OS=$(OS) ($(ARCH)) | GPU Mode=$(if $(filter 1,$(ENABLE_CUDA)),Enabled (NVIDIA GPU),Disabled (CPU/Mac))"
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
	@echo "  make test-pc          Run full pipeline test (YOLOP + Control + Traffic Light + RViz)"
	@echo "                        Options: DEVICE=cpu|cuda|mps  VIDEO=/path/to/video.mp4"
	@echo "  make test-tl          Test traffic light detection & distance estimation"
	@echo "  make test-yolop       Test YOLOP lane segmentation only"
	@echo "  make test-control     Test lane detection + Pure Pursuit control"
	@echo "  make vgui             Run Web Verification GUI (open http://localhost:8090)"
	@echo "  make stop-nodes       Kill all running ROS 2 nodes inside container"
	@echo ""
	@echo "🌐 [Web Simulator & UIs]"
	@echo "  make rosbridge        Start rosbridge WebSocket server on port 9090"
	@echo "  make open-sim (sim)   Open 3D Web Simulator in browser (http://localhost:8000)"
	@echo "  make open-rviz (gui)  Open RViz2 Web Display in browser (http://localhost:8080)"
	@echo "  make rqt-graph        Open rqt_graph in browser GUI (http://localhost:8080)"
	@echo "  make rqt              Open full rqt dashboard in browser GUI (http://localhost:8080)"
	@echo "  make open-vgui        Open Web Verification GUI in browser (http://localhost:8090)"
	@echo ""
	@echo "🏎️ [Real Vehicle Operations]"
	@echo "  make bringup-hw       Launch hardware nodes only"
	@echo "  make bringup-all      Launch hardware + full autonomous stack"
	@echo "  make teleop           Run keyboard teleoperation"
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
	$(DOCKER_COMPOSE) build --build-arg ENABLE_CUDA=$(ENABLE_CUDA)

rebuild:
	$(DOCKER_COMPOSE) build --no-cache --build-arg ENABLE_CUDA=$(ENABLE_CUDA)

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
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

build-pkg:
	@if [ -z "$(PKG)" ]; then \
		echo "[ERROR] Please specify PKG. Example: make build-pkg PKG=oit_navigation"; \
		exit 1; \
	fi
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		$(DOCKER_COMPOSE) up -d; \
	fi
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select $(PKG) --symlink-install"

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

test-yolop:
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		$(DOCKER_COMPOSE) up -d; \
	fi
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c \
		"source /opt/ros/humble/setup.bash && source install/setup.bash && \
		 ros2 launch oit_navigation yolop_video_test.launch.py \
		 $(if $(VIDEO),video_path:=$(VIDEO),) \
		 use_device:=$(DEVICE)"

test-control:
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		$(DOCKER_COMPOSE) up -d; \
	fi
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c \
		"source /opt/ros/humble/setup.bash && source install/setup.bash && \
		 ros2 launch oit_navigation video_test.launch.py \
		 $(if $(VIDEO),video_path:=$(VIDEO),) \
		 use_device:=$(DEVICE) \
		 traffic_light:=false \
		 rviz:=true"

vgui verification-gui:
	@if ! $(DOCKER_COMPOSE) ps --services --filter "status=running" | grep -q "$(SERVICE_NAME)"; then \
		$(DOCKER_COMPOSE) up -d; \
	fi
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c \
		"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run oit_navigation verification_gui"

stop-nodes kill:
	$(DOCKER_COMPOSE) exec $(SERVICE_NAME) bash -c \
		"pkill -9 -f 'ros2|rviz2|video_publisher|yolop_lane_detector|bev_pure_pursuit_node|traffic_light|robot_state_publisher|joint_state_publisher' || true"

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

# ------------------------------------------------------------------------------
# Real Vehicle Operations
# ------------------------------------------------------------------------------

bringup-hw:
	bash bash/1_bringup_hardware.sh

bringup-all:
	bash bash/3_bringup_all_nodes.sh

teleop:
	bash bash/teleop_keyboard.sh
