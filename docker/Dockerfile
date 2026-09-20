FROM ros:humble

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS 2 desktop packages and essential development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
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
    python3-colcon-common-extensions \
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
    ros-humble-can-msgs \
    ros-humble-cv-bridge \
    ros-humble-diagnostic-updater \
    ros-humble-geographic-msgs \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-joy \
    ros-humble-nmea-msgs \
    ros-humble-robot-localization \
    ros-humble-ros2-socketcan \
    ros-humble-rqt-runtime-monitor \
    ros-humble-teleop-twist-joy \
    ros-humble-teleop-twist-keyboard \
    ros-humble-vision-msgs \
    ros-humble-xacro \
    ros-humble-rosbridge-server \
    ros-humble-rosbridge-suite \
    kmod \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Install AI / Perception and Hardware Communication dependencies
# numpy/opencv-python are re-pinned in the second command too: without it, pip lets
# ultralytics (and separately onnx, needed by export_tensorrt.py) pull in numpy>=2 +
# a matching opencv-python, which breaks the apt-installed cv_bridge and matplotlib
# (compiled against numpy 1.x) with "_ARRAY_API not found".
# PyTorch は --index-url (not --extra-index-url) で入れること。
# --extra-index-url だと PyPI も候補に残り、pip は「より新しいバージョン」を選ぶため
# PyPI のデフォルト wheel (torch 2.14.0+cu130 など) が勝ってしまう。CUDA 13 ビルドは
# ドライバ r580 以上を要求するので、車載/開発機の driver 535 (CUDA 12.2) では
# "The NVIDIA driver on your system is too old (found version 12020)" で
# torch.cuda.is_available() == False になる。
# TORCH_CUDA_CHANNEL は driver に合わせて選ぶ (cu118: driver>=520 / cu121: >=530 / cu124: >=550)。
ARG ENABLE_CUDA=0
ARG TORCH_CUDA_CHANNEL=cu121
ARG TORCH_VERSION=2.5.1
ARG TORCHVISION_VERSION=0.20.1
# CUDA_APT_VERSION/TENSORRT_CU must match TORCH_CUDA_CHANNEL (cu121 -> 12-1/cu12,
# cu118 -> 11-8/cu11) - see the ENABLE_CUDA branch below for why these are needed.
ARG CUDA_APT_VERSION=12-1
ARG TENSORRT_CU=cu12
ARG TENSORRT_VERSION=10.8.0.43
RUN pip3 install --no-cache-dir \
    "setuptools<70.0.0" \
    "numpy<2.0.0" \
    && if [ "$ENABLE_CUDA" = "1" ]; then \
    echo "[INFO] Installing PyTorch ${TORCH_VERSION} with CUDA support (${TORCH_CUDA_CHANNEL}) and TensorRT..." && \
    pip3 install --no-cache-dir \
    "torch==${TORCH_VERSION}" "torchvision==${TORCHVISION_VERSION}" \
    --index-url https://download.pytorch.org/whl/${TORCH_CUDA_CHANNEL} && \
    python3 -c "import torch, sys; v=torch.version.cuda or ''; print('[INFO] installed torch', torch.__version__, 'cuda', v); sys.exit(0 if v.startswith('12.') or v.startswith('11.') else 1)" && \
    ( \
    echo "[INFO] Installing CUDA Toolkit dev headers/libs (${CUDA_APT_VERSION}) for TensorRT/PyCUDA..." && \
    curl -fsSL -o /tmp/cuda-keyring.deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i /tmp/cuda-keyring.deb && rm -f /tmp/cuda-keyring.deb && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    cuda-cudart-dev-${CUDA_APT_VERSION} \
    cuda-driver-dev-${CUDA_APT_VERSION} \
    cuda-profiler-api-${CUDA_APT_VERSION} \
    cuda-nvcc-${CUDA_APT_VERSION} \
    libcurand-dev-${CUDA_APT_VERSION} \
    && rm -rf /var/lib/apt/lists/* && \
    echo "[INFO] Installing PyCUDA (needs the headers above - the pip nvidia-cuda-runtime wheel alone lacks cudaProfiler.h/curand.h/crt/host_config.h)..." && \
    CPATH=/usr/local/cuda/include \
    LIBRARY_PATH=/usr/local/cuda/targets/x86_64-linux/lib:/usr/local/cuda/targets/x86_64-linux/lib/stubs \
    pip3 install --no-cache-dir pycuda && \
    echo "[INFO] Installing TensorRT (${TENSORRT_CU} ${TENSORRT_VERSION})..." && \
    pip3 install --no-cache-dir \
    "tensorrt-${TENSORRT_CU}==${TENSORRT_VERSION}" \
    "tensorrt-${TENSORRT_CU}-libs==${TENSORRT_VERSION}" \
    "tensorrt-${TENSORRT_CU}-bindings==${TENSORRT_VERSION}" \
    ) || echo "[WARN] TensorRT/PyCUDA installation failed - lane_detector (backend=yolop) will fall back to plain PyTorch inference at runtime (use_tensorrt has no effect)."; \
    else \
    echo "[INFO] Installing PyTorch with CPU support..." && \
    pip3 install --no-cache-dir \
    "torch==${TORCH_VERSION}" "torchvision==${TORCHVISION_VERSION}" \
    --index-url https://download.pytorch.org/whl/cpu; \
    fi \
    && pip3 install --no-cache-dir \
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

# Setup noVNC default index
RUN ln -s /usr/share/novnc/vnc.html /usr/share/novnc/index.html 2>/dev/null || true


# Initialize and update rosdep
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
    fi

# Create non-root user 'rosuser' with sudo privileges
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME 2>/dev/null || true \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash 2>/dev/null || true \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Update rosdep as rosuser
USER $USERNAME
RUN rosdep update
USER root

# Configure system-wide ROS 2 environment for all shells (interactive, non-interactive, and login)
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
    && echo "if [ -f /aiformula_machine/install/setup.bash ]; then source /aiformula_machine/install/setup.bash; fi" >> /etc/bash.bashrc \
    && mkdir -p /etc/profile.d \
    && echo "source /opt/ros/humble/setup.bash" > /etc/profile.d/ros.sh \
    && echo "if [ -f /aiformula_machine/install/setup.bash ]; then source /aiformula_machine/install/setup.bash; fi" >> /etc/profile.d/ros.sh \
    && chmod +x /etc/profile.d/ros.sh

# Configure .bashrc for rosuser (prepend to ensure it runs even for non-interactive subshells)
RUN sed -i '1i source /opt/ros/humble/setup.bash' /home/$USERNAME/.bashrc \
    && echo "if [ -f /aiformula_machine/install/setup.bash ]; then source /aiformula_machine/install/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "eval \"\$(register-python-argcomplete3 ros2)\"" >> /home/$USERNAME/.bashrc \
    && echo "eval \"\$(register-python-argcomplete3 colcon)\"" >> /home/$USERNAME/.bashrc \
    && echo "export ROS_DOMAIN_ID=100" >> /home/$USERNAME/.bashrc

# Set default shell environment variables
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/humble
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3
ENV PATH=/opt/ros/humble/bin:$PATH
# /usr/local/cuda/targets/.../lib carries the real libcurand.so etc. installed
# above for PyCUDA/TensorRT (see ENABLE_CUDA build arg). Harmless no-op on a
# CPU/Mac build where that directory doesn't exist - ld.so just skips it.
# NOTE: intentionally NOT adding its lib/stubs subdir here - that stub libcuda.so
# is a build-time link-only placeholder, not something to accidentally load at
# runtime instead of the real driver's libcuda.so.
ENV LD_LIBRARY_PATH=/usr/local/cuda/targets/x86_64-linux/lib:/opt/ros/humble/lib:$LD_LIBRARY_PATH
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH
ENV CMAKE_PREFIX_PATH=/opt/ros/humble:$CMAKE_PREFIX_PATH
ENV DISPLAY=:1
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV QT_X11_NO_MITSHM=1


# Set up entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Workspace directory
WORKDIR /aiformula_machine
RUN chown -R $USERNAME:$USERNAME /aiformula_machine 2>/dev/null || true

USER $USERNAME

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
