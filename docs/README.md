# Test4 Navigation 文档

这套文档按“先理解系统，再实现算法，最后接成 ROS 节点”的顺序组织。每篇只回答一个问题，读完后都能对应到一个实现任务或验证步骤。

## 阅读顺序

| 顺序 | 文档 | 要解决的问题 | 读完后做什么 |
|---:|---|---|---|
| 1 | `Part_of_Nav_Overview.md` | 导航系统由什么组成，数据怎样流动 | 知道每个包的职责和任务边界 |
| 2 | `Part_of_Map.md` | 坐标、TF、栅格索引和代价图怎样对应 | 完成坐标换算、`toGridMap` 和地图理解 |
| 3 | `Part_of_AStar.md` | 怎样在代价栅格上求合法可执行路径 | 完成 `astar_planner.cpp` |
| 4 | `Part_of_PathTracking.md` | 怎样让差速底盘沿路径运动 | 完成 `pure_pursuit.cpp` |
| 5 | `Part_of_NavExecutor.md` | 怎样把地图、TF、规划和控制接成节点 | 完成节点和 launch |
| 6 | `Part_of_Foxglove.md` | 怎样在 3D 面板中点击发布目标点 | 向导航系统发送目标点 |

## 建议工作顺序

```text
环境自检
  -> 只跑仿真
  -> 坐标与栅格换算
  -> A* 单元测试
  -> 纯追踪单元测试
  -> nav_executor 集成
  -> launch
  -> 分别重启系统并运行三个运行时校验
```

不要在整套系统启动失败时同时修改 A*、控制器和节点。先用单元测试验证纯算法，再用 Foxglove 观察话题和运行时校验定位集成问题。

## 文件和任务的对应关系

| 文件 | 是否修改 | 任务 |
|---|---|---|
| `src/nav_planner/src/astar_planner.cpp` | 修改 | 栅格换算和 A* |
| `src/nav_controller/src/pure_pursuit.cpp` | 修改 | 纯追踪 |
| `src/nav_executor/src/nav_executor_node.cpp` | 修改 | ROS 集成 |
| `src/nav_launcher/launch/nav.launch.py` | 新建 | 一键启动 |
| `nav_common/`、`nav_sim/`、`map_server/` | 不修改 | 已提供的基础设施和裁判 |
