# Test4 Navigation 项目报告

## 一、项目概述

本项目在 ROS 2 Jazzy 环境下完成一条完整的 2D 导航链路：

```text
占据栅格代价图 + 目标点 + TF 位姿
        ↓
      A* 全局规划
        ↓
     纯追踪路径跟踪
        ↓
   差速底盘速度指令
```

实现内容包括：

- `nav_planner`：栅格坐标换算与 8 邻域 A* 全局规划
- `nav_controller`：纯追踪控制器
- `nav_executor`：地图、目标、TF、规划和控制之间的节点集成
- `nav_launcher`：用 `component_container_mt` 一键启动完整系统

---

## 二、实现内容

### 1. 栅格坐标换算

实现 `gridToWorld` 和 `worldToGrid`：

```text
世界 → 栅格：
col = floor((x - origin_x) / resolution)
row = floor((y - origin_y) / resolution)

栅格 → 世界（格子中心）：
x = origin_x + (col + 0.5) * resolution
y = origin_y + (row + 0.5) * resolution
```

注意点：

- `col` 对应 x，`row` 对应 y
- 世界 → 栅格必须用 `floor`
- 越界返回 `false`
- 输出路径使用格子中心，而不是原始连续坐标

### 2. A* 全局规划

实现 `planPath`：

- 8 邻域搜索
- 直方向代价 `resolution`，对角代价 `resolution * sqrt(2)`
- 欧氏距离作为启发函数
- 障碍判断使用 `value >= occupied_threshold`，默认 `99`
- 对角移动禁止穿墙角
- 使用 `priority_queue`、`g_score`、`came_from`
- 弹出时跳过过期节点
- 到达目标后沿 `came_from` 回溯并反转
- 起点或终点在障碍内、越界或不可达时返回空路径
- 起点和终点在同一格时返回该格中心一个点

### 3. 纯追踪控制

实现 `computeCommand`：

- 空路径和到达目标时返回零速
- 从最近路径点开始向前找前瞻点
- 将前瞻点变换到车体系：

```text
lx =  cos(yaw) * dx + sin(yaw) * dy
ly = -sin(yaw) * dx + cos(yaw) * dy
```

- 曲率：

```text
L² = lx² + ly²
kappa = 2 * ly / L²
```

- 角速度：

```text
w = v * kappa
```

- 角速度限幅到 `[-max_angular_velocity, max_angular_velocity]`
- 接近目标时按剩余距离线性减速
- 处理除零、NaN 和非有限值

### 4. 节点集成

在 `nav_executor_node.cpp` 中实现：

- `toGridMap`：`OccupancyGrid` → `GridMap`
- `lookupPose`：查询 `map -> base_link` TF，提取平移和 yaw
- `plan`：调用 `planPath`，发布 `/nav_executor/global_path`
- `goalCallback`：接收 `/nav_goal`，检查 `frame_id`，触发规划
- `tick`：状态机、到达判定、纯追踪、加减速限制、发布 `cmd_vel`

安全策略：

- 没有目标、没有路径、TF 查询失败时都发布零速
- 规划失败时清空旧路径并发布空路径
- 线速度变化受最大加速度和最大减速度限制
- 到达后清空路径、清空目标并停车

### 5. launch

新建 `nav.launch.py`：

- 用 `component_container_mt` 加载：
  - `map_server::MapServerNode`
  - `nav_sim::RobotSimNode`
  - `nav_executor::NavExecutorNode`
- 三个组件分别加载各自的 YAML 参数
- 独立启动 `foxglove_bridge`
- 提供 `use_foxglove` 和 `foxglove_port` launch 参数

---

## 三、测试结果

### 单元测试

```text
nav_planner + nav_controller
20 tests, 0 errors, 0 failures, 0 skipped
```

### 默认目标 `(20.18, 9.98)`

| 校验 | 结果 |
|---|---|
| `verify_planner.py` | 通过，338 个路径点，首尾偏差 0.007 m |
| `verify_reach.py` | 通过，20.5 s，最近距离 0.190 m |
| `verify_tracking.py` | 通过，20.5 s，最大横向误差 0.083 m |

### 换目标 `(21.0, 3.0)`

| 校验 | 结果 |
|---|---|
| `verify_planner.py` | 通过，332 个路径点 |
| `verify_reach.py` | 通过，19.0 s，最近距离 0.287 m |
| `verify_tracking.py` | 通过，19.0 s，最大横向误差 0.070 m |

说明实现没有写死默认目标。

---

## 四、三个问题

### 1. 学到了什么？是否能看出这个任务想考察什么？做的过程中压力如何？

通过这个项目，我了解了导航系统从地图到控制的工作流程和具体工程实现：`map_server` 根据原始地图生成代价图；`nav_executor` 订阅代价图，用 A* 搜索路径；控制器确定最近点和前瞻点，通过 `map -> base_link` 把前瞻点变换到车体系，计算曲率 `kappa` 和角速度，并用

```text
v = v_max * min(1, d_goal / slowdown_distance)
```

计算线速度，最终把 `cmd_vel` 发给 `robot_sim`；到达判定使用 `goal_tolerance`。

此外，我还扩展学习了代价图的生成方式：Nav2 内部使用 `253` 表示 inscribed、`254` 表示 lethal，膨胀区用 `252 * exp(-cost_scaling_factor * (d - inscribed_radius))` 计算；本项目为了适配 `OccupancyGrid` 的 `0..100` 范围，使用 `99` 表示 inscribed、`100` 表示 lethal，膨胀区用 `99 * exp(-24.0 * (d - 0.1))` 计算并限制在 `1..98`。

压力方面：整体感觉还好。

### 2. 你做了哪些尝试？哪些方法行得通，哪些不行？为什么？

**行得通的做法：**

- 先完成坐标换算，再写 A*。坐标换算虽然简单，但它是后面所有路径的基础；先把 `gridToWorld` 和 `worldToGrid` 的单元测试跑通，后面定位问题会容易很多。
- A* 使用 8 邻域、欧氏启发、对角代价 `sqrt(2)`，并在对角移动时检查两个直邻格。这样既能让路径比较自然，也能避免穿墙角。
- 纯追踪先处理空路径和到达判定，再找最近点和前瞻点，最后算曲率和角速度。把“停车条件”放在曲率计算之前，可以避免车到终点附近时做无意义的除法。
- 用 `colcon test` 分别验证 `nav_planner` 和 `nav_controller`，再启动完整系统跑三个 verify。分层验证比一上来就启动整套系统更容易定位问题。
- 换目标 `(21.0, 3.0)` 复测三项校验，确认没有写死默认目标。

**走过弯路或失败的做法：**

- 一开始直接拿 odom 位姿理解导航，忽略了 `map -> odom` 的非单位变换。后来通过对比 `/nav_sim/ground_truth`、`/nav_sim/odom` 和 `tf2_echo map base_link`，才真正理解为什么必须查 `map -> base_link`。
- 试过目标 `(23.0, 3.0)`，规划器返回不可达。这个点在地图灰度图上是自由格，但离真实障碍太近，落在代价图 `>= 99` 的膨胀区，所以被正确拒绝。这让我理解了“地图自由”和“规划可通行”不是一回事。
- 第一次 `verify_tracking` 失败，一开始怀疑是控制参数或路径问题，后来发现是旧 launch 没退干净，导致两个同名 `nav_executor` 同时存在，目标消息被其中一个收到。清理进程并重启 ROS daemon 后通过。

**录屏后的改进：线性安全代价**

- 第一次录屏时发现规划出的路径离障碍物很近。排查后确认不是栅格换算或 TF 的问题，而是 A* 只把 `>=99` 当作禁行，步进代价只有几何距离，代价图中的梯度值 `1..98` 完全没有参与搜索；因此路径会在所有“可通行”格子中选几何最短的一条，自然贴着 inscribed 边界走。
- 为了平衡安全距离与物理距离，在 `astar_planner.cpp` 的步进代价中引入线性安全代价：

```text
step_cost = base_distance × (1 + k × cell_cost / 100),  k = 2.0
```

  其中 `base_distance` 是 `resolution` 或 `resolution × √2`，`cell_cost` 是目标格的代价值。
- 该写法保证实际步进代价始终不小于几何距离，因此原来的欧氏启发函数仍然可采纳；代价值越高的格子被加价越多，A* 会主动选择走廊中间的低代价格。
- 默认目标下的效果：最小离障距离从 `0.141 m` 提升到 `0.212 m`，路径平均代价值从 `7.48` 降到 `0.18`，最大代价值从 `37` 降到 `7`；路径长度仅从 `19.51 m` 增加到 `19.77 m`（约 1.4%）。
- 最终提交版本采用加入线性安全代价的版本；第一次录屏和原始版本保留下来作为对比。

### 3. 你有使用 AI 辅助吗？如果有，你是如何使用的？

有使用 AI 辅助。

主要用途：

- 理解概念：坐标系、TF、栅格换算、A* 的启发函数、纯追踪的曲率和符号、QoS、组件和 launch。
- 代码 review：检查 `ly` 符号、实际前瞻距离 `L`、障碍阈值、越界判断、过期节点和加速度限制。
- 调试：分析 `verify_tracking` 第一次失败的原因，检查 ROS 图中是否有重复节点，确认 `/nav_goal` 的订阅者数量。
- 整理测试结果和报告结构。

使用原则是：不直接把代码当成最终答案，而是先自己理解每一步，再运行单元测试和运行时校验确认。AI 给出的代码也经过实际编译和测试，最终以 Ubuntu 上的 `colcon test` 和三个 verify 脚本的结果为准。

---

## 五、不足与改进

- 当前是静态地图、静态障碍和单目标场景，没有实现动态重规划和局部避障。
- 纯追踪在急弯处仍有切角倾向，本项目依靠代价图膨胀留出安全余量。
- 规划失败时 `tick` 会每个周期重试，日志会比较多；可以增加重规划冷却时间。
- 没有实现更复杂的路径平滑或安全代价搜索，当前路径在几何上不是最短，但满足合法性和可执行性。
- 更接近实际系统的做法是由定位模块动态发布 `map -> odom`，本项目仿真中它是静态非单位变换，主要用于验证 TF 使用是否正确。
- 建议课程后续增加真实场景资料选读与拓展，例如 Nav2 架构、AMCL/SLAM、动态障碍和实车调试等内容。

---

## 六、附：运行方式

```bash
cd ~/test4
source /opt/ros/jazzy/setup.bash
./compile.sh
source install/setup.bash

ros2 launch nav_launcher nav.launch.py

# 另开终端
ros2 topic pub --once /nav_goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 20.18, y: 9.98}}"

ros2 run nav_launcher verify_planner.py
ros2 run nav_launcher verify_reach.py
ros2 run nav_launcher verify_tracking.py
```

Foxglove：

```text
ws://<ubuntu-ip>:8765
```

3D 面板 Display frame 设为 `map`，添加：

```text
/map_server/global_cost_map
/nav_executor/global_path
/nav_sim/ground_truth_path
```
