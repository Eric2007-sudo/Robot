<h1 align="center">Part of Validation</h1>

这篇文档讲怎样验证导航链路，以及不同错误现象分别对应哪一层。验证应该从纯算法到整套系统逐层进行，不能只看“车有没有到”。

## 一、环境检查

在项目根目录执行：

```bash
bash check_env.sh
```

它只检查，不修改系统。先看 ROS 发行版、`colcon`、`ninja`、ROS 依赖、`foxglove_bridge` 和端口。环境检查没有通过时，后面的编译错误没有诊断价值。

### 参数从文件到节点

本题参数经过以下链路：

```text
src/*/config/*.yaml
  -> colcon install / symlink-install
  -> launch 中的 parameters=[yaml_path]
  -> 节点 declare_parameter<T>("参数名")
```

| 问题 | 常见现象 | 检查方法 |
|---|---|---|
| 参数路径错误 | launch 报参数文件不存在 | 检查 `get_package_share_directory()` 拼出的路径 |
| 新建 launch 未重新编译 | `ros2 launch` 找不到文件 | 重新运行 `./compile.sh` |
| 参数名不一致 | 节点启动时参数异常 | 对照 YAML 和 `declare_parameter` |
| 参数类型不一致 | 参数转换失败 | 浮点写 `0.1`，整数写 `1` |
| 修改后节点未重启 | 行为仍是旧配置 | 停止并重新启动节点 |

## 二、只跑仿真

```bash
source install/setup.bash
ros2 launch nav_sim sim_only.launch.py
```

这个 launch 不启动导航节点，只启动地图服务器、差速底盘仿真和 Foxglove bridge。正常现象：

- 地图服务器发布 `/map_server/global_cost_map`；
- TF 树为 `map -> odom -> base_link`；
- `/nav_sim/odom`、`/nav_sim/ground_truth`、`/nav_sim/ground_truth_path` 和 `/nav_sim/map_velocity` 发布；
- 机器人停在起点，没有计划路径，也不会自动移动；实际轨迹 Path 此时只有起点样本。

检查命令：

```bash
ros2 node list
ros2 topic echo /map_server/global_cost_map --once
ros2 topic echo /nav_sim/map_velocity --once
ros2 run tf2_ros tf2_echo map base_link
```

地图消息应为 `560 x 300`、分辨率 `0.05`。如果地图上下颠倒，检查地图加载的行序；如果 TF 不完整，先修环境或仿真，不要修改 A*。

## 三、单元测试

完成 A* 后：

```bash
colcon test --packages-select nav_planner
colcon test-result --verbose
```

完成纯追踪后：

```bash
colcon test --packages-select nav_controller
colcon test-result --verbose
```

测试失败时先看失败测试名：

| 测试类型 | 通常说明 |
|---|---|
| 坐标换算 | origin、row/col 或 floor 写错 |
| 路径合法性 | 坐标换算、障碍阈值、穿墙角或路径段碰撞 |
| 对角合法性 | 没检查穿墙角 |
| 控制安全性 | 坐标变换、转向方向、速度和角速度是否有限且受约束 |
| 角速度限幅 | 没有 clamp |

不要在单元测试失败时直接启动整套系统。纯函数可以在不依赖 TF 和 ROS 运行时的情况下先修好。

需要查看测试的直接输出时，可以使用：

```bash
colcon test --packages-select nav_planner --event-handlers console_direct+
colcon test-result --verbose
```

如果结果中出现旧包的失败记录，应先重新运行对应包的测试，再判断当前实现是否失败。

## 四、Foxglove

完整 launch 启动后，bridge 默认地址是：

```text
ws://localhost:8765
```

在 3D 面板中把 Display frame 设为 `map`，添加：

```text
/map_server/global_cost_map
/nav_executor/global_path
/nav_sim/ground_truth
/nav_sim/ground_truth_path
/nav_sim/map_velocity
```

这里有两条不同的 Path：`/nav_executor/global_path` 是 A* 算出的**计划路径**；
`/nav_sim/ground_truth_path` 是 robot_sim 从真值位姿累积出的**实际行驶轨迹**。
`/nav_sim/ground_truth` 只是当前时刻的一个 PoseStamped 点，不是历史轨迹。

`/nav_sim/map_velocity` 是 `geometry_msgs/msg/TwistStamped`，`header.frame_id=map`。
其中 `twist.linear.x/y` 是地图坐标系下的实际速度；`/nav_executor/cmd_vel.linear.x`
则是底盘坐标系下的前向控制指令，两者不要直接当成同一个量比较。

正确现象：地图上能同时看到计划路径和实际轨迹；车运行时实际轨迹逐渐贴近计划路径，
到达后两条线大致重合。代价图应显示真实障碍、近障碍的高代价区域和更远的梯度区域。

`global_path` 使用 transient-local QoS，并在活动路径期间周期重发；Foxglove 晚于规划连接时，
也能收到计划线。真值轨迹按距离采样并限制最大点数，运行时间长也不会无限增长。

Foxglove 只提供观察，不替代校验脚本。画面看起来正常，不代表路径合法或没有碰撞。


## 五、三个运行时校验

整套系统启动后，另开终端并 source 工作空间：

### 1. 规划器

```bash
ros2 run nav_launcher verify_planner.py
```

检查：

- 路径是否存在；
- 起点和终点是否正确；
- 路径点和连续路径段是否都避开障碍；
- 对角移动是否穿墙角。

规划器失败时先查 `astar_planner.cpp`，不要先改控制器。

### 2. 闭环到达

```bash
ros2 run nav_launcher verify_reach.py
```

检查机器人是否在限定时间内到达目标，以及是否进入真实障碍。它使用真值判断碰撞：`100` 才是碰撞，`99` 和 `1..98` 是代价图语义，不等于真实撞墙。

路径正确但到不了时，依次检查 TF、目标和 `cmd_vel`，再检查纯追踪。

### 3. 跟踪质量

```bash
ros2 run nav_launcher verify_tracking.py
```

检查实际轨迹和全局路径的横向误差。默认 `1.0 m` 只是质量参考值，不是到达硬约束；车能到达但误差明显偏大，通常是前瞻点选择、车体系变换、`ly` 符号或角速度限幅问题。

## 六、按现象定位

```text
没有地图
  -> map_server / 参数文件 / 话题名 / QoS

地图上下颠倒
  -> msgpack 行序和加载器是否重复翻转

地图正常，没有路径
  -> /nav_goal、TF、A* 起终点合法性

路径不绕障或明显绕远
  -> occupied_threshold、启发函数、对角代价、穿墙角检查

路径正确，车不动
  -> tick、纯追踪到达判定、cmd_vel 发布

车原地打转
  -> ly 符号、map 系位姿、角速度方向

车到达但撞墙
  -> 路径是否使用代价图、规划阈值是否为 99、控制器是否切角过大

车到达但横向误差大
  -> lookahead、实际 L、曲率系数和角速度限幅
```

## 七、最终验收

通过标准：

- `nav_planner` 和 `nav_controller` 单元测试全部通过；
- 三个 verify 脚本全部通过；
- 连续运行至少 3 分钟不崩溃；
- Foxglove 中地图、路径和真实轨迹对齐；
- 更换目标点后仍然可以重新规划和到达。

更换目标点很重要。写死目标、路径、地图尺寸或某个固定方向，默认目标可能碰巧通过，换目标后会暴露问题。

建议从干净构建开始做最终检查：

```bash
rm -rf build install log
./compile.sh
source install/setup.bash
colcon test --packages-select nav_planner nav_controller
colcon test-result --verbose
ros2 launch nav_launcher nav.launch.py
```

启动后再分别运行三个校验脚本。这样可以同时验证源码构建、资源安装、参数加载和运行时通信。
