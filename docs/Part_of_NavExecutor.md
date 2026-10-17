<h1 align="center">Part of Navigation Executor</h1>

全局规划和路径跟踪算法都是纯函数。它们单独通过单元测试，还不能让机器人运动。本篇说明 `nav_executor` 怎样接收地图和目标、查询机器人位姿、发布路径，并在定时器中持续发送速度指令。

## 一、节点的输入和输出

| 数据 | 来源 | 用途 |
|---|---|---|
| `/map_server/global_cost_map` | `map_server` | A* 的地图输入 |
| `/nav_goal` | 校验脚本或 Foxglove | 触发一次规划 |
| `map -> base_link` | TF | 得到 map 系当前位姿 |
| `/nav_executor/global_path` | `nav_executor` | 发布规划结果 |
| `/nav_sim/ground_truth_path` | `robot_sim` | 发布实际行驶轨迹，供 Foxglove 显示 |
| `/nav_executor/cmd_vel` | `nav_executor` | 驱动仿真底盘 |

所有规划相关的位置都必须在 `map` 系表达：地图、目标、路径和当前机器人位姿。

启动后可以用以下命令确认接口：

```bash
ros2 node info /nav_executor
ros2 topic info /map_server/global_cost_map
ros2 topic info /nav_goal
```

如果节点存在但订阅数量为 0，检查话题名和参数文件；如果订阅存在但没有消息，检查发布者和 QoS。

## 二、成员状态

节点需要跨回调保存以下状态：

```text
costmap_       最近收到的代价图
has_costmap_   是否已经收到地图
goal_          当前目标点
has_goal_      是否存在目标
path_          当前全局路径
```

地图是 transient local 发布的，晚启动的订阅者也能收到最近一张地图；但代码仍然要检查 `has_costmap_`，不能假设回调一定先于目标回调执行。

地图订阅使用 `KeepLast(1).transient_local()`。深度为 1 表示只保留最新地图，`transient_local` 保证地图服务器先发布、导航节点后启动时仍能收到地图。

## 三、五个 TODO 的职责

### 1. `toGridMap`

只做消息字段拷贝：宽、高、分辨率、原点和数据。

```cpp
nav_common::GridMap map;
map.width = static_cast<int>(msg.info.width);
map.height = static_cast<int>(msg.info.height);
map.resolution = msg.info.resolution;
map.origin_x = msg.info.origin.position.x;
map.origin_y = msg.info.origin.position.y;
map.data = msg.data;
return map;
```

`OccupancyGrid::data` 与 `GridMap::data` 都是 `int8` 数组。不要在这里重新翻转、交换宽高或做坐标变换。

### 2. `lookupPose`

查询：

```cpp
lookupTransform("map", "base_link", tf2::TimePointZero)
```

平移直接取 `transform.translation.x/y`。姿态用 `tf2::Quaternion` 和 `tf2::Matrix3x3::getRPY` 提取 yaw。TF 尚未准备好时会抛 `tf2::TransformException`，要捕获并返回 `false`，不能让异常退出节点。

### 3. `plan`

推荐顺序：

```text
没有地图或目标 -> 返回
查询当前 map 系位姿 -> 失败则返回
调用 planPath(costmap_, 当前位姿, goal_)
路径为空 -> 清空当前路径并打印警告
路径非空 -> 保存 path_ 并 publishPath()
```

目标回调和控制循环都可能触发 `plan()`，所以 `plan()` 不能依赖某个回调一定先发生。

规划失败时要清空旧路径，否则机器人可能继续沿上一条路径运行。空路径也可能是起点、终点位于障碍区或两点不可达，应通过日志区分这些情况。

### 4. `goalCallback`

目标消息是 `PointStamped`，本题要求目标使用 `map` frame。保存 `point.x/y`，设置 `has_goal_=true`，然后触发 `plan()`。

不要使用目标姿态作为底盘最终朝向；本题的到达判定只检查位置。

### 5. `tick`

定时器每个周期执行：

```text
没有目标       -> 发布零速度
没有路径       -> 尝试规划，再发布零速度
TF 查询失败    -> 发布零速度
距离目标小于 tol -> 发布零速度，清空任务状态
其他情况       -> computeCommand(path_, pose, params) 得到目标速度，限制加减速后发布 Twist
```

安全原则是：任何前置数据不完整时都发零速度，而不是沿用上一条速度。


## 四、路径发布

`global_path` 是 `nav_msgs/Path`，header frame 必须是 `map`。路径中的每个点也是 `map` 系的 `PoseStamped`。本题只需要设置 position 和一个合法的单位姿态，例如 `orientation.w=1.0`。

如果 frame 写成 `odom`，Foxglove 中路径可能看起来存在，但和地图、真值不能重合。

要区分两条线：`/nav_executor/global_path` 是规划器给出的**计划路线**；`/nav_sim/ground_truth_path` 是 `robot_sim` 把真值位姿按距离采样后累积的**实际行驶路线**。`/nav_sim/ground_truth` 本身只是每一时刻的一个 PoseStamped，不保存历史，不能单独画成轨迹。

`global_path` 在路径有效期间会周期重发，并使用 transient-local QoS。这样 Foxglove 晚于规划连接，也能收到计划路线；到达后保留最后一条计划路线，便于和实际轨迹对照。

## 五、速度发布

控制器返回：

```text
Command.v -> 目标 geometry_msgs/Twist.linear.x，再经过加减速限制
Command.w -> geometry_msgs/Twist.angular.z
```

差速底盘只使用这两个分量。其余 Twist 分量保持 0。

## 六、launch 文件

需要按 `src/nav_launcher/launch/README.md` 创建 `nav.launch.py`。完整系统由地图服务器、仿真、导航组件和 Foxglove bridge 构成；当前项目使用组件容器承载三个 ROS 组件，bridge 独立运行。

参数路径不要写死。使用：

```python
from ament_index_python.packages import get_package_share_directory

share = get_package_share_directory('map_server')
params = os.path.join(share, 'config', 'map_server_params.yaml')
```

新建 launch 文件后要重新编译一次，之后修改已有 launch 内容通常不需要重新编译，因为项目使用了 `--symlink-install`。

参数路径应由包的 share 目录生成：

```python
params = os.path.join(
  get_package_share_directory('map_server'),
  'config', 'map_server_params.yaml')
```

不要使用当前工作目录拼相对路径。`ros2 launch` 可以从任意目录执行，当前工作目录不保证是项目根目录。

## 七、启动顺序和排错

完整启动：

```bash
ros2 launch nav_launcher nav.launch.py
```

另开终端发布目标：

```bash
ros2 topic pub --once /nav_goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 20.18, y: 9.98}}"
```

观察三个输出：

```bash
ros2 topic echo /nav_executor/global_path --once
ros2 topic echo /nav_executor/cmd_vel
ros2 topic echo /nav_sim/ground_truth
```

| 现象 | 优先检查 |
|---|---|
| 没有路径 | 地图是否收到、目标 frame、TF 是否存在 |
| 路径有但车不动 | `cmd_vel` 是否持续发布、控制器是否返回零速度 |
| 路径和地图错位 | `map -> base_link` 是否查询正确、Path frame 是否为 `map` |
| 车到不了目标 | 到达阈值、纯追踪前瞻点、角速度限幅 |

### 话题类型分工

目标输入和路径/位姿显示使用不同的消息类型，各自职责固定，不要混用：

| 话题 | 消息类型 | 用途 |
|---|---|---|
| `/nav_goal` | `geometry_msgs/msg/PointStamped` | Foxglove 点击或验证脚本发送目标点；只需要 `x/y` 和坐标系 |
| `/nav_sim/ground_truth` | `geometry_msgs/msg/PoseStamped` | 仿真发布机器人当前真实位置和朝向 |
| `/nav_executor/global_path` | `nav_msgs/msg/Path` | 发布规划路径；其中每个 `poses[]` 元素是 `PoseStamped` |

集成阶段可以按下面的顺序检查：

```bash
ros2 topic echo /map_server/global_cost_map --once
ros2 topic pub --once /nav_goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 20.18, y: 9.98}}"
ros2 topic echo /nav_executor/global_path --once
ros2 topic hz /nav_executor/cmd_vel
```

四条命令分别确认地图、目标、规划结果和控制输出。每一步失败时只检查对应的上游模块。
