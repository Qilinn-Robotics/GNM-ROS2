# 基于官方 ROS Humble 全桌面版镜像 (Ubuntu 22.04)
FROM osrf/ros:humble-desktop-full

# 设置非交互前端
ENV DEBIAN_FRONTEND=noninteractive

# 1. 配置 Ubuntu 清华源 (可选)
RUN sed -i 's/archive.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list

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
    # 导航相关 (Nav2) - 虽然 GNM 不直接用 Nav2，但可能有共用消息或工具
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# 3. 配置 pip 清华源并升级 pip
RUN pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple && \
    pip3 install --no-cache-dir --upgrade pip

# 4. 安装 ViNT/GNM 所需的 Python 依赖
# 对应 Ubuntu 22.04 (Python 3.10)
RUN pip3 install --no-cache-dir --ignore-installed \
    torch \
    torchvision \
    numpy \
    matplotlib \
    pyyaml \
    einops \
    vit_pytorch \
    prettytable \
    # rospkg 在 ROS 2 中通常不需要，但为了兼容旧脚本可能还需要
    rospkg \
    # === 新增依赖 (ViNT/NoMaD) ===
    efficientnet_pytorch \
    diffusers==0.11.1 \
    "huggingface_hub<0.14.0" \
    git+https://github.com/ildoonet/pytorch-gradual-warmup-lr.git \
    # === 通用依赖 ===
    scipy \
    scikit-learn \
    wandb \
    termcolor \
    opencv-python \
    pandas \
    # ROS 2 Python 额外工具
    transforms3d

# 5. 安装本地包 (ViNT 和 Diffusion Policy)
# 将源码复制到 /opt 目录下并安装，这样即使 /code 被覆盖挂载，库依然可用
COPY src/visualnav_transformer/train /opt/vint_train
RUN pip3 install -e /opt/vint_train

COPY third_party/diffusion_policy /opt/diffusion_policy
RUN pip3 install -e /opt/diffusion_policy

# 6. 配置 ROS 2 环境自动加载
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 7. 设置工作目录
WORKDIR /code

# 启动命令
CMD ["bash"]
