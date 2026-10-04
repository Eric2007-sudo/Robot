#!/bin/bash
# 编译整个工作空间。
#   --symlink-install : 安装时用软链接，改 YAML / launch / 脚本无需重新 build
#   -G Ninja          : 用 Ninja 生成器（需 sudo apt install ninja-build）
#                       切换生成器要先删掉 build/ install/ log/
set -e

cd "$(dirname "$0")"

# Ninja 缺失时 CMake 的报错是 "CMAKE_C_COMPILER not set"，看起来像编译器问题，
# 很容易查错方向。这里提前拦一下。
if ! command -v ninja > /dev/null; then
    echo "找不到 ninja。本脚本用 -G Ninja 作为 CMake 生成器，先装上："
    echo "    sudo apt install ninja-build"
    echo "完整的环境检查：bash check_env.sh"
    exit 1
fi

# source 过旧 install/setup.bash 后若删库重建，环境变量里会残留已失效的工作区路径，
# colcon 启动时会刷一屏 prefix_path WARNING。这里先剔除不存在的路径再构建。
sanitize_prefix_path() {
    local var_name="$1"
    local value="${!var_name}"
    if [ -z "$value" ]; then
        return 0
    fi
    local parts out="" p
    IFS=':' read -ra parts <<< "$value"
    for p in "${parts[@]}"; do
        if [ -e "$p" ]; then
            out="${out:+$out:}$p"
        fi
    done
    export "$var_name=$out"
}
sanitize_prefix_path CMAKE_PREFIX_PATH
sanitize_prefix_path AMENT_PREFIX_PATH

colcon build --symlink-install --cmake-args -G Ninja
