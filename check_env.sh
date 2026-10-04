#!/bin/bash
# 环境自检。遇到问题先跑这个，再把输出贴出来提问。
#
#   bash check_env.sh
#
# 它只检查和报告，不会动你的系统。缺什么会直接给出安装命令。

cd "$(dirname "$0")"

pass=0
fail=0

ok()   { echo "  [ok]   $1"; pass=$((pass+1)); }
bad()  { echo "  [缺失] $1"; echo "         修复：$2"; fail=$((fail+1)); }
warn() { echo "  [注意] $1"; }

echo "=================================================================="
echo "  Locator 环境自检"
echo "=================================================================="

# ---- 系统与发行版 ----
echo
echo "系统"
if [ -r /etc/os-release ]; then
    . /etc/os-release
    ok "$PRETTY_NAME"
else
    warn "读不到 /etc/os-release，无法确认 Ubuntu 版本"
fi

if [ -z "$ROS_DISTRO" ]; then
    bad "ROS_DISTRO 没有值，说明没 source 过 ROS 2" \
        "source /opt/ros/<发行版>/setup.bash；长期生效的话写进 ~/.bashrc"
    echo
    echo "ROS 2 环境都没加载，后面的检查没有意义，先解决这一条。"
    echo "=================================================================="
    exit 1
fi
ok "ROS_DISTRO = $ROS_DISTRO"

case "$ROS_DISTRO" in
    jazzy)   ok "Jazzy 是文档的主线版本" ;;
    lyrical) warn "Lyrical 可以用。启动时 component_container_mt 会打一条弃用警告，不影响运行" ;;
    *)       warn "$ROS_DISTRO 没有验证过。文档以 Jazzy 为主线，Lyrical 也跑过" ;;
esac

# ---- 构建工具 ----
echo
echo "构建工具"
for tool in colcon cmake g++; do
    if command -v $tool > /dev/null; then
        ok "$tool: $(command -v $tool)"
    else
        case $tool in
            colcon) bad "colcon" "sudo apt install python3-colcon-common-extensions" ;;
            *)      bad "$tool" "sudo apt install build-essential cmake" ;;
        esac
    fi
done

if command -v ninja > /dev/null; then
    ok "ninja: $(ninja --version)"
else
    bad "ninja（compile.sh 指定了 -G Ninja，没有它第一步就会失败）" \
        "sudo apt install ninja-build"
fi

# ---- ROS 2 包 ----
echo
echo "ROS 2 包"
for pkg in rclcpp rclcpp_components tf2 tf2_ros tf2_geometry_msgs \
           geometry_msgs sensor_msgs nav_msgs visualization_msgs rclpy; do
    if ros2 pkg prefix $pkg > /dev/null 2>&1; then
        ok "$pkg"
    else
        bad "$pkg" "sudo apt install ros-\$ROS_DISTRO-$(echo $pkg | tr '_' '-')"
    fi
done

if ros2 pkg prefix foxglove_bridge > /dev/null 2>&1; then
    ok "foxglove_bridge"
else
    bad "foxglove_bridge（可视化要用，不装的话 launch 会报找不到包）" \
        "sudo apt install ros-\$ROS_DISTRO-foxglove-bridge"
fi

# ---- rosdep ----
echo
echo "rosdep"
if command -v rosdep > /dev/null || command -v rosdepc > /dev/null; then
    RD=$(command -v rosdep || command -v rosdepc)
    ok "$(basename $RD): $RD"
    if [ -d src ]; then
        if $RD check --from-paths src --ignore-src > /tmp/.locator_rosdep 2>&1; then
            ok "src/ 下所有包的依赖都已满足"
        else
            echo "  [注意] rosdep 报告还有依赖没装："
            sed 's/^/         /' /tmp/.locator_rosdep | head -10
            echo "         修复：$(basename $RD) install --from-paths src --ignore-src -y"
            fail=$((fail+1))
        fi
        rm -f /tmp/.locator_rosdep
    fi
else
    warn "没有 rosdep。它能按 package.xml 一次装齐依赖，建议装上："
    echo "         sudo apt install python3-rosdep"
    echo "         sudo rosdep init && rosdep update"
    echo "         国内网络下 rosdep update 容易失败，fishros 提供了 rosdepc 可以替代"
fi

# ---- 可视化 ----
echo
echo "可视化"
if ss -ltn 2>/dev/null | grep -q ':8765'; then
    warn "8765 端口已被占用。可能有残留的 foxglove_bridge 还在跑："
    echo "         pkill -f foxglove_bridge"
else
    ok "8765 端口空闲"
fi

# ---- 残留进程 ----
echo
echo "残留进程"
leftover=$(pgrep -af 'component_containe[r]|instal[l]/locator' 2>/dev/null | wc -l)
if [ "$leftover" -gt 0 ]; then
    warn "有 $leftover 个本项目相关的进程还在跑。上次没退干净会让这次结果不对："
    pgrep -af 'component_containe[r]|instal[l]/locator' 2>/dev/null | sed 's/^/         /'
    echo "         清理：pkill -f component_container"
else
    ok "没有残留进程"
fi

# ---- 小结 ----
echo
echo "=================================================================="
if [ $fail -eq 0 ]; then
    echo "  全部通过（$pass 项）。接下来："
    echo "    ./compile.sh"
    echo "    source install/setup.bash"
    echo "    ros2 launch locator_launcher sim_only.launch.py"
else
    echo "  $fail 项需要处理，$pass 项通过。按上面的修复命令逐条解决。"
fi
echo "=================================================================="

[ $fail -eq 0 ]
