#!/bin/bash
# 这是一个用于在容器内设置开发环境的脚本
# 它会以可编辑模式 (-e) 安装本地包，这样你在宿主机修改代码，容器内会立即生效

PROJECT_ROOT="/code/GNM-ROS2"

echo "Installing local packages in editable mode..."

# 安装 Diffusion Policy
if [ -d "$PROJECT_ROOT/third_party/diffusion_policy" ]; then
    if [ -f "$PROJECT_ROOT/third_party/diffusion_policy/setup.py" ] || [ -f "$PROJECT_ROOT/third_party/diffusion_policy/pyproject.toml" ]; then
        echo "Installing diffusion_policy..."
        pip3 install -e "$PROJECT_ROOT/third_party/diffusion_policy"
    else
        echo "Warning: diffusion_policy directory exists but no setup.py/pyproject.toml found. Did you forget to update submodules?"
    fi
else
    echo "Error: $PROJECT_ROOT/third_party/diffusion_policy not found!"
fi

echo "Setup complete!"
