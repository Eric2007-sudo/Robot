# launch 目录

`sim_only.launch.py` 是给你的示例，只启动仿真，不需要改。

**你要写的是 `locator_launcher.launch.py`，现在还不存在。**

## 需要做到

一条命令把整套系统拉起来：

```bash
ros2 launch locator_launcher locator_launcher.launch.py
```

- 四个仿真节点和 locator 放进**同一个** `ComposableNodeContainer`
  - 容器用 `component_container_mt`（多线程执行器）
  - 每个组件加 `use_intra_process_comms: True`
- 仿真节点加载 `locator_sim` 的 `sim_params.yaml`
- locator 加载 `locator` 的 `locator_params.yaml`
- 启动 `foxglove_bridge`
- 支持命令行参数：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `use_foxglove` | `true` | 是否启动 bridge |
| `foxglove_port` | `8765` | WebSocket 端口 |
| `override_params` | `locator_launcher/config/empty.yaml` | 额外的参数覆盖文件 |

`override_params` 指向的 yaml 要叠加在**所有**节点的默认参数之后，也就是每个组件的
`parameters` 写成 `[默认yaml, override]`。**仿真和 locator 两边都要加**，否则安装量
两边不一致，结果全错。

## 几个必须知道的点

**为什么所有节点要在一个容器里。** 四个仿真节点靠一个进程内共享的起点对齐仿真时间。
拆成多个进程不会报错，只是各节点描述的世界不一致。locator 也放进去，是为了用上进程内
通信。原因见 `docs/Part_of_ROS2.md` 第三节。

**参数文件路径不能写死。** 用
`get_package_share_directory('locator_sim')` 拿 share 目录再拼路径。

**`plugin` 填什么。** 填 C++ 类的完整名字（含命名空间），和源文件末尾
`RCLCPP_COMPONENTS_REGISTER_NODE(...)` 里的一致。仿真四个节点的类名可以从
`src/locator_sim/CMakeLists.txt` 的 `PLUGIN` 一行找到，locator 的在
`src/locator/CMakeLists.txt` 里。

**`foxglove_bridge` 是系统装的独立节点**，`package` 和 `executable` 都叫
`foxglove_bridge`，有一个 `port` 参数。

**改完要编译一次。** `--symlink-install` 只对编译时已存在的文件建软链接，新建的
launch 文件要重新 `./compile.sh` 一次才会出现在 `install/` 里。

## 参照

`sim_only.launch.py` 可以直接跑，也可以照着改。它和你要写的完整版的差别是：少了
locator 那个组件，没有命令行参数，没有 `override_params`。

launch 的完整说明见 `docs/Part_of_ROS2.md` 第四节。
