# launch 文件要求

在这个目录里创建 `nav.launch.py`，把整套导航系统一次拉起来。

## 必须做到

1. 使用 `rclcpp_components/component_container_mt` 和 `ComposableNode` 启动地图服务器、仿真与导航组件：
   - `map_server`：加载 `map_server` 的 `config/map_server_params.yaml`，发布 `/map_server/global_cost_map`
   - `robot_sim`：`nav_sim` 包的差速底盘仿真，加载 `nav_sim` 的 `config/sim_params.yaml`
   - `nav_executor`：加载 `nav_executor` 的 `config/nav_params.yaml`
2. 三个组件分别加载 `map_server_params.yaml`、`sim_params.yaml` 和 `nav_params.yaml`。
3. 独立启动 `foxglove_bridge`，端口默认 `8765`；建议提供 `use_foxglove` 和 `foxglove_port` launch 参数。

## 提示

- 参数文件路径用 `get_package_share_directory(...)` 拼，不要写死绝对路径。
- 容器使用 `rclcpp_components/component_container_mt`；组件 plugin 分别为
  `map_server::MapServerNode`、`nav_sim::RobotSimNode`、`nav_executor::NavExecutorNode`。
- 组件 `package` / `plugin` 名称要与各包 CMake 注册的 component plugin 一致。
- 可以参考 `nav_sim/launch/sim_only.launch.py` 的写法，它只起仿真、不起导航。
- Foxglove bridge 的端口建议做成 launch 参数（默认 8765），可参考 Test1/Test2 的写法。

## 验收

```bash
source install/setup.bash
ros2 launch nav_launcher nav.launch.py
```

启动后：
- `ros2 node list` 能看到 map_server、robot_sim、nav_executor、foxglove_bridge；
- `ros2 topic echo /map_server/global_cost_map`（或 `--once`）能看到代价图；
- 另开终端跑 `ros2 run nav_launcher verify_planner.py` 等三个脚本能打印"通过"。
