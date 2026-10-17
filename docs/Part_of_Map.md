<h1 align="center">Part of Map</h1>

这篇文档说明导航使用的地图、坐标系、栅格索引和代价图。A* 的输入不是一张普通图片，而是一张带有世界坐标和通行代价的 `OccupancyGrid`。读完后应能完成 `toGridMap`、`lookupPose`、`gridToWorld` 和 `worldToGrid` 四个函数

## 一、导航中的坐标系

本题的 TF 树是：

```text
map ── 静态 ──> odom ── 动态 ──> base_link
```

| 坐标系 | 含义 | 相关数据 |
|---|---|---|
| `map` | 全局地图坐标系 | 地图、目标、全局路径、真值 |
| `odom` | 从机器人起点积分出的局部坐标系 | `/nav_sim/odom` |
| `base_link` | 机器人本体坐标系 | 底盘中心和车头方向 |

`map -> odom` 不是恒等变换。仿真默认给它设置了平移和旋转，用于检验导航节点是否正确使用 TF。不能把 `/nav_sim/odom` 中的位姿直接当成 `map` 系位姿。

导航节点需要查询：

```cpp
auto tf = tf_buffer_->lookupTransform(
    frames::MAP, frames::BASE_LINK, tf2::TimePointZero);
```

第一个参数是结果所在的坐标系，第二个参数是被查询的坐标系。返回的变换是 `map -> base_link`，可以直接取平移和 yaw。

## 二、OccupancyGrid 的布局

地图消息的主要字段是：

```text
info.width       列数，沿世界 x 增大
info.height      行数，沿世界 y 增大
info.resolution  一个格子的边长，单位 m
info.origin      row=0,col=0 格子左下角的世界坐标
data             一维行主序数组
```

数据索引为：

```cpp
data[row * width + col]
```

本题地图尺寸为 `560 x 300`，分辨率为 `0.05 m`，覆盖约 `28 m x 15 m`。

需要区分两套顺序：

- 世界坐标写成 `(x, y)`；
- 栅格索引写成 `(row, col)`；
- `col` 对应 `x`，`row` 对应 `y`。

## 三、世界坐标与栅格坐标

给定世界点 `(x, y)`，包含它的格子为：

$$
col = \left\lfloor\frac{x-origin_x}{resolution}\right\rfloor,
\qquad
row = \left\lfloor\frac{y-origin_y}{resolution}\right\rfloor
$$

实现时使用 `floor`，不能使用 `round`：

```cpp
int col = static_cast<int>(std::floor((x - map.origin_x) / map.resolution));
int row = static_cast<int>(std::floor((y - map.origin_y) / map.resolution));
if (!map.inBounds(row, col)) return false;
*col_out = col;
*row_out = row;
return true;
```

A* 输出的是格子中心：

$$
x = origin_x + (col+0.5)resolution,
\qquad
y = origin_y + (row+0.5)resolution
$$

例如 `origin=(0,0)`、`resolution=0.05` 时，格子 `(col=40,row=20)` 的中心是 `(2.025,1.025)`。

## 四、地图文件和方向

地图服务器读取 `maps/RMUC202605.msgpack`。每格包含一个地形标签，本题只保留静态障碍：

```text
OBSTACLE -> 100
其他标签  -> 0
```


`toGridMap` 只做消息字段转换：

```text
msg.info.width              -> map.width
msg.info.height             -> map.height
msg.info.resolution         -> map.resolution
msg.info.origin.position.x  -> map.origin_x
msg.info.origin.position.y  -> map.origin_y
msg.data                    -> map.data
```

## 五、代价图的作用

原始占据图只表示自由空间和真实障碍。真实机器人有尺寸，如果直接在原始障碍边界上规划，机器人中心虽然没有进入障碍格，车身仍可能碰到障碍。因此地图服务器会对障碍做膨胀，把机器人近似为质点进行规划。

本题地图服务器发布：

```text
/map_server/global_cost_map
```

消息类型为 `nav_msgs/OccupancyGrid`，代价值为：

| 数值 | 含义 |
|---:|---|
| `100` | 真实障碍，lethal |
| `99` | 满代价区域，规划器不进入 |
| `1..98` | 连续梯度区域，数值越大表示离障碍越近 |
| `0` | 自由空间 |

规划器使用阈值判断：代价 `>=99` 的格子不可进入；碰撞判定只把 `100` 视为真实障碍。`1..98` 保存了距离障碍的连续信息，用于可视化、参数检查和后续需要代价大小的算法。

## 六、连续梯度膨胀

对每个格子，地图服务器先计算它到最近真实障碍的欧氏距离 `d`，再按参数赋值：

```text
d <= full_cost_radius_m                  -> 99
full_cost_radius_m < d <= cutoff_radius_m -> 1..98，指数衰减
 d > cutoff_radius_m                     -> 0
```

衰减区的计算形式为：

$$
cost = 99\exp\left(-decay\_rate\_per\_m\,(d-full\_cost\_radius\_m)\right)
$$

结果限制在 `1..98`，再转换为 `int8`。真实障碍始终保持 `100`。

当前参数为：

```yaml
global_map:
  inflation:
    full_cost_radius_m: 0.1
    cutoff_radius_m: 0.3
    decay_rate_per_m: 24.0
```

连续梯度不会改变本题 A* 的障碍阈值，A* 仍然使用 `occupied_threshold=99`。它能在地图上显示距离障碍的远近，并保留代价信息。

## 七、地图验证

只启动仿真：

```bash
source install/setup.bash
ros2 launch nav_sim sim_only.launch.py
```

检查地图消息：

```bash
ros2 topic echo /map_server/global_cost_map --once
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

Foxglove 中将 Display frame 设置为 `map`，添加 `/map_server/global_cost_map`。

## 八、本题对应的实现

| 知识点 | 对应位置 |
|---|---|
| TF 位姿 | `nav_executor::lookupPose` |
| OccupancyGrid 转换 | `nav_executor::toGridMap` |
| 世界/栅格换算 | `gridToWorld`、`worldToGrid` |
| 原始地图加载 | `map_server/src/map_loader.cpp` |
| 连续梯度膨胀 | `map_server/src/costmap_inflation.cpp` |
| A* 消费代价图 | `planPath` 中的 `occupied_threshold` |
