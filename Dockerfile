# 基于官方 ROS Humble 全桌面版镜像 (Ubuntu 22.04)
FROM osrf/ros:humble-desktop-full

# 设置非交互前端
ENV DEBIAN_FRONTEND=noninteractive

# 1. 配置 Ubuntu 镜像源 (可选)
RUN sed -i 's/[a-z]\+\.ubuntu\.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    sed -i 's/ports.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list

# 2. 安装基础工具和 ROS 2 依赖
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-tk \
    git \
    nano \
    wget \
    curl \
    tmux \
    # ROS 2 通用依赖
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-usb-cam \
    ros-humble-v4l2-camera \
    ros-humble-joy \
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    ros-humble-twist-mux \
    ros-humble-robot-localization \
    ros-humble-xacro \
    # 导航相关 (Nav2)
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# 3. 配置 pip 镜像源并升级 pip
RUN pip3 config set global.index-url https://mirrors.aliyun.com/pypi/simple && \
    pip3 install --no-cache-dir --upgrade pip

# 4. 安装 ViNT/GNM 所需的 Python 依赖
# 对应 Ubuntu 22.04 (Python 3.10)
RUN pip3 install --no-cache-dir --ignore-installed \
    torch \
    torchvision \
    numpy==1.24.3 \
    matplotlib \
    pyyaml \
    einops \
    vit_pytorch \
    prettytable \
    rospkg \
    efficientnet_pytorch \
    diffusers==0.11.1 \
    "huggingface_hub<0.14.0" \
    git+https://github.com/ildoonet/pytorch-gradual-warmup-lr.git \
    # === 通用依赖 ===
    scipy \
    scikit-learn \
    wandb \
    termcolor \
    pandas \
    # ROS 2 Python 额外工具
    transforms3d

# 5. 设置开发环境路径 (可选)
ENV PYTHONPATH="${PYTHONPATH}:/code/GNM-ROS2/src:/code/GNM-ROS2/third_party/diffusion_policy"

# 6. 配置 ROS 2 环境和开发包自动加载
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f /code/GNM-ROS2/scripts/install_gnm_dep.sh ] && /code/GNM-ROS2/scripts/install_gnm_dep.sh" >> /root/.bashrc

# 7. 设置工作目录
WORKDIR /code

# 启动命令
CMD ["bash"]
