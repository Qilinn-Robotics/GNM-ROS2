#!/bin/bash
CONTAINER_NAME="gnm-test-ros2"

echo "Starting container: $CONTAINER_NAME"

xhost +
docker run -it --rm --privileged --net=host \
    --name $CONTAINER_NAME \
    --ipc=host \
    --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="DISPLAY=$DISPLAY" \
    --env="WAYLAND_DISPLAY=$WAYLAND_DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${HOME}/codes:/code:rw" \
    gnm:humble \
    bash