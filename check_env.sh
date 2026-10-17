#!/usr/bin/env bash
# 检查导航任务需要的环境。只报告问题，不修改系统。
# 遇到任何环境方面的问题，先跑它，把输出一起贴出来提问。
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

pass=0
fail=0
warn_count=0

ok() {
  echo "  [ok]   $1"
  pass=$((pass + 1))
}

bad() {
  echo "  [缺失] $1"
  echo "         修复：$2"
  fail=$((fail + 1))
}

warn() {
  echo "  [注意] $1"
  warn_count=$((warn_count + 1))
}

echo "============================================================"
echo "  Test4_Navigation 环境自检"
echo "============================================================"

echo
echo "系统与 ROS 2"
if [[ -r /etc/os-release ]]; then
  source /etc/os-release
  ok "$PRETTY_NAME"
else
  warn "无法读取 /etc/os-release"
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  bad "ROS_DISTRO 未设置，当前终端没有 source ROS 2" \
    "source /opt/ros/jazzy/setup.bash 或 source /opt/ros/lyrical/setup.bash"
else
  ok "ROS_DISTRO=$ROS_DISTRO"
  case "$ROS_DISTRO" in
    jazzy|lyrical) ok "$ROS_DISTRO 在项目验证范围内" ;;
    *) warn "$ROS_DISTRO 未在本项目中验证，优先使用 jazzy 或 lyrical" ;;
  esac
fi

echo
echo "构建工具"
for tool in bash python3 g++ cmake colcon ninja ros2; do
  if command -v "$tool" >/dev/null 2>&1; then
    ok "$tool: $(command -v "$tool")"
  else
    case "$tool" in
      colcon) bad "colcon" "sudo apt install python3-colcon-common-extensions" ;;
      ninja) bad "ninja" "sudo apt install ninja-build" ;;
      ros2) bad "ros2" "先 source /opt/ros/<distro>/setup.bash" ;;
      *) bad "$tool" "sudo apt install build-essential cmake python3" ;;
    esac
  fi
done

echo
echo "ROS 2 依赖"
ros_packages=(
  rclcpp rclcpp_components nav_msgs geometry_msgs
  tf2 tf2_ros ament_index_cpp rclpy launch launch_ros foxglove_bridge
  ament_cmake_gtest
)
for pkg in "${ros_packages[@]}"; do
  if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    ok "$pkg"
  else
    if [[ -n "${ROS_DISTRO:-}" ]]; then
      deb="ros-${ROS_DISTRO}-${pkg//_/-}"
      bad "$pkg" "sudo apt install $deb"
    else
      bad "$pkg" "先 source ROS 2，再安装对应 ros-<distro>- 包"
    fi
  fi
done

echo
echo "系统库"
if [[ -f /usr/include/yaml-cpp/yaml.h ]]; then
  ok "yaml-cpp"
else
  bad "yaml-cpp（nav_sim 读地图 YAML 要用）" "sudo apt install libyaml-cpp-dev"
fi

echo
echo "工作空间"
if [[ -d src ]]; then
  ok "src/ 存在"
else
  bad "src/ 不存在" "在项目根目录运行本脚本"
fi

if [[ -f build/CMakeCache.txt ]]; then
  generator="$(sed -n 's/^CMAKE_GENERATOR:INTERNAL=//p' build/CMakeCache.txt | head -n 1)"
  if [[ "$generator" == Ninja* ]]; then
    ok "build/ 使用 Ninja"
  else
    warn "build/ 使用 $generator；首次切换 Ninja 前请删除 build install log"
  fi
fi

echo
echo "端口与残留进程"
if command -v ss >/dev/null 2>&1; then
  if ss -ltn 2>/dev/null | grep -q ':8765'; then
    warn "8765 端口已被占用，可能有旧的 foxglove_bridge 进程"
  else
    ok "8765 端口空闲"
  fi
fi

leftover=$(pgrep -af 'map_publishe[r]|robot_si[m]|nav_executo[r]|foxglove_bridg[e]' 2>/dev/null | wc -l)
if [[ "$leftover" -gt 0 ]]; then
  warn "有 $leftover 个本项目相关进程还在跑，可能影响本次结果："
  pgrep -af 'map_publishe[r]|robot_si[m]|nav_executo[r]|foxglove_bridg[e]' 2>/dev/null | sed 's/^/         /'
  echo "         清理：pkill -f map_publisher; pkill -f robot_sim; pkill -f nav_executor; pkill -f foxglove_bridge"
else
  ok "没有残留进程"
fi

echo
echo "============================================================"
echo "  通过：$pass    缺失：$fail    注意：$warn_count"
if [[ "$fail" -eq 0 ]]; then
  echo "  环境检查通过。接下来可以运行："
  echo "    ./compile.sh"
  echo "    source install/setup.bash"
  echo "    ros2 launch nav_sim sim_only.launch.py"
else
  echo "  请先处理上面的缺失项。"
fi
echo "============================================================"

[[ "$fail" -eq 0 ]]
