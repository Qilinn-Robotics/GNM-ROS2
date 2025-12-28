#!/bin/bash

CONFIG_PATH="./gnm_nav.rviz"

echo "正在启动 RViz2 并加载配置: ${CONFIG_PATH}"

rviz2 -d "${CONFIG_PATH}"
