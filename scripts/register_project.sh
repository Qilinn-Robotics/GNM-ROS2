#!/bin/bash
# 脚本功能：将项目的 src 目录和 third_party/diffusion_policy 目录注册到虚拟环境的 PYTHONPATH 中

# 获取项目根目录的绝对路径
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 检查虚拟环境是否存在
VENV_PATH="${PROJECT_ROOT}/.venv"
if [ ! -d "$VENV_PATH" ]; then
    echo "错误: 未找到虚拟环境 $VENV_PATH"
    echo "请先创建虚拟环境，例如: python3 -m venv .venv"
    exit 1
fi

# 寻找虚拟环境中的 site-packages 目录
# 考虑不同 Python 版本的路径（如 lib/python3.10/site-packages）
SITE_PACKAGES=$(find "$VENV_PATH/lib" -name "site-packages" -type d | head -n 1)

if [ -z "$SITE_PACKAGES" ]; then
    echo "错误: 无法在虚拟环境中定位 site-packages 目录"
    exit 1
fi

# 定义 .pth 文件路径
PTH_FILE="$SITE_PACKAGES/gnm_project.pth"

echo "正在生成 $PTH_FILE ..."

# 写入路径
# 1. 项目的 src 目录
echo "${PROJECT_ROOT}/src" > "$PTH_FILE"
# 2. third_party 中的 diffusion_policy 目录
echo "${PROJECT_ROOT}/third_party/diffusion_policy" >> "$PTH_FILE"

echo "完成！"
echo "现在虚拟环境已包含以下路径:"
cat "$PTH_FILE"
