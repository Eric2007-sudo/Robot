<h1 align="center">Part of ROS 2</h1>

这份文档是 Test1 那份 `Part_of_ROS2.md` 的续篇。节点的结构、构造函数的顺序、yaml 参数、订阅发布、launch 的基础写法，那份都讲过，这里不再重复。

本文讲本题新增的四件事：QoS 怎样决定两个节点能不能通信、执行器和回调组怎样决定回调能不能并行、为什么要把多个节点放进同一个进程，以及更复杂的 launch 写法。

读完之后，你应该能写出 `LocatorNode` 构造函数里的两个订阅和回调组，以及完整的 `locator_launcher.launch.py`。

## 先读这个

> Test1 的 **`Part_of_ROS2.md`**（在 Test1 仓库的 `docs/` 下）
>
> 尤其是第二节"构造函数的顺序"、第三节"yaml 参数怎么走到代码里"、第四节"回调是怎么被调用的"、第七节"launch 文件"。本文假设你已经写过一个能跑的节点。

## 一、QoS 通信

### QoS 的概念

QoS（Quality of Service）是发布者和订阅者各自声明的一组通信策略。两端的策略**兼容**时才能建立连接，不兼容时 ROS 2 只打一条警告，然后一条消息都不传。节点照常运行，回调永远不触发。

本题用到三项策略：

| 策略 | 取值 | 含义 |
|---|---|---|
| reliability | `reliable` | 保证送达，丢了会重传 |
| | `best_effort` | 尽力而为，丢了就丢了 |
| durability | `volatile` | 只发给当前已经连上的订阅者 |
| | `transient_local` | 发布者保留最近几条，后连上的订阅者也能收到 |
| history / depth | `keep_last(N)` | 队列里最多保留 N 条 |

### 兼容规则

规则只有一条：**订阅者要求的不能比发布者承诺的更多。**

下面是实测结果（发布者发 5 条，或者先发 1 条再建订阅）：

| 发布者 | 订阅者 | 能否连接 | 收到 |
|---|---|---|---|
| reliable | reliable | 能 | 全部 |
| best_effort | **reliable** | **不能**，打印 incompatible QoS | 0 条 |
| reliable | best_effort | 能 | 全部 |
| best_effort | best_effort | 能 | 全部 |
| volatile | **transient_local** | **不能**，打印 incompatible QoS | 0 条 |
| transient_local，先发后订阅 | volatile | 能 | 订阅之前发的那条**收不到** |
| transient_local，先发后订阅 | transient_local | 能 | 订阅之前发的那条**能收到** |

前两组最常出现问题：发布者是 best_effort，订阅者写成了 reliable，两端谈不拢。

### 本题的 QoS

| 话题 | 发布者 | 为什么这样选 |
|---|---|---|
| `locator_sim/imu/data` | `SensorDataQoS().keep_last(50)` | 高频传感器数据，丢一帧下一帧马上就到，不值得重传 |
| `locator_sim/detector/target_pose` | `QoS(10)` | reliable，每一帧观测都要处理 |
| `/tf` | tf2 默认 | reliable |
| `/tf_static` | tf2 默认 | reliable + transient_local，后启动的节点也能拿到 |
| `locator_sim/markers/static` | `QoS(1).transient_local()` | 静态 marker 只发一次 |

`SensorDataQoS` 是 rclcpp 预设的传感器配置：best_effort、volatile、keep_last(5)。`.keep_last(50)` 把队列深度改成 50。

`rclcpp::QoS(10)` 是另一个常用写法：reliable、volatile、keep_last(10)。

订阅 IMU 时，如果写成 `rclcpp::QoS(10)`，就是上表第二行的情况。实测 locator 启动时打印

```
[WARN] [locator]: New publisher discovered on topic '/locator_sim/imu/data', offering incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```

之后 IMU 回调一次都不触发，云台 TF 永远不发布，`verify_accuracy` 一个样本都配不上。这条警告只打一次，混在启动日志里很容易错过。

### 查看两端的 QoS

```bash
ros2 topic info -v /locator_sim/imu/data
```

会列出这个话题上每个发布者和订阅者的 QoS：

```
Node name: virtual_imu
Endpoint type: PUBLISHER
  Reliability: BEST_EFFORT
  History (Depth): KEEP_LAST (50)
  Durability: VOLATILE
```

订阅端的 QoS 怎么选，最稳妥的办法就是先用这条命令看发布端，再决定。

## 二、执行器与回调组

### 回调由谁调用

Test1 讲过：消息到达后进入队列，`spin` 从队列里取出就绪的回调去执行。负责"取出并执行"的就是执行器（executor）。

执行器有两种常用的：

- `SingleThreadedExecutor`：一个线程，所有回调一个接一个地执行
- `MultiThreadedExecutor`：一个线程池，多个回调可以同时执行。线程数默认等于 CPU 核数（最少 2 个）

多线程执行器并不是让所有回调都随便并行。哪些回调可以同时跑，由**回调组**决定。

### 回调组

每个回调（订阅、定时器、服务）都属于一个回调组。回调组有两种：

- `MutuallyExclusive`（互斥）：组内的回调一次只能执行一个。同一个回调也不会和自己同时执行
- `Reentrant`（可重入）：组内的回调可以同时执行，同一个回调也可以有多个实例同时在跑

不同组之间互不限制，只要执行器有空闲线程，就能并行。

**节点有一个默认回调组，类型是 MutuallyExclusive。** 不指定回调组时，所有回调都进这个默认组，所以即使用了多线程执行器，同一个节点的回调也是串行的。

### 三者的关系

| 执行器 | 回调组 | 两路回调能否并行 |
|---|---|---|
| 单线程 | 任意 | 不能。只有一个线程，分了组也没用 |
| 多线程 | 都在默认组 | 不能。默认组是互斥的 |
| 多线程 | 两个独立的组 | 能 |

### 本题为什么要分组

IMU 回调每 2 ms 来一次，target 回调每 16.7 ms 来一次。两者串行时，target 回调里任何一点耗时都会让 IMU 回调排队。

`Part_of_TF.md` 第五节讲过阻塞等待 TF 为什么会锁死。和执行器结合起来看，实测结果是：

| 查询方式 | 执行器 | 回调组 | 结果 |
|---|---|---|---|
| 非阻塞 | 单线程 | 默认 | 通过 |
| 非阻塞 | 多线程 | 两个独立组 | 通过 |
| 阻塞 100 ms | 多线程 | 两个独立组 | 60 秒 × 2 次，通过 |
| 阻塞 100 ms | 单线程 | 默认 | 60 秒 × 3 次，全部锁死 |
| 阻塞 100 ms | 单线程 | 两个独立组 | 有时通过，有时锁死。60 秒 × 3 次全部锁死 |

几点结论：

1. **非阻塞查询是根本。** 按 `Part_of_TF.md` 的写法做，执行器和回调组怎么配都能通过
2. 分组只有配合多线程执行器才起作用。单线程下分组和不分组没有区别
3. 锁死有随机性。短时间跑通不代表没问题，这也是为什么不能只凭"跑过了"判断代码对不对

既然非阻塞时都能过，为什么题目还要求分组？因为非阻塞只能保证"不会等"，不能保证回调本身不耗时。真实系统里 target 回调可能要做滤波、匹配，耗时几毫秒很正常。分组之后，一路回调慢了不会拖累另一路。这是一种防御性设计：现在不分组也能跑，但改动之后可能就不行了。

**回调组这一项运行结果看不出来，批改时会看代码。**

### 写法

```cpp
// 成员变量
rclcpp::CallbackGroup::SharedPtr group_a_;
rclcpp::CallbackGroup::SharedPtr group_b_;

// 构造函数里
group_a_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
group_b_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

rclcpp::SubscriptionOptions options_a;
options_a.callback_group = group_a_;

sub_a_ = create_subscription<std_msgs::msg::Int32>(
    "topic_a",
    rclcpp::QoS(10),
    [this](const std_msgs::msg::Int32::SharedPtr msg) { on_a(msg); },
    options_a                    // 第四个参数
);
```

第四个参数不传时，订阅进默认组。

回调组的 `SharedPtr` 必须作为成员变量保存下来。只存在构造函数的局部变量里，构造函数结束后回调组会被销毁。

### 选 MutuallyExclusive 还是 Reentrant

两种都能让两路回调并行，区别在于**同一路回调**能不能和自己并行。

问自己一个问题：回调里有没有读写成员变量？如果同一个回调的两个实例同时在跑，同时读写同一个成员变量，会发生什么？

`LocatorNode` 里已经给好的 `publish_marker` 会往 `trail_points_` 里 `push_back`。想清楚这一点，再决定用哪种。报告里有一道题问的就是这个。

### 容器的执行器

本题的节点跑在组件容器里，执行器由容器决定：

| 容器 | 执行器 |
|---|---|
| `component_container` | 单线程 |
| `component_container_mt` | 多线程 |

所以回调组要起作用，launch 里必须用 `component_container_mt`。

**发行版差异**：在 Lyrical 上启动 `component_container_mt` 会打印

```
This executable is deprecated and will be removed in M-turtle.
Use 'component_container --executor-type multi-threaded' instead.
```

不影响运行。Jazzy 的 `component_container` 还没有 `--executor-type` 参数，所以本题统一用 `component_container_mt`，两个发行版都能跑。

## 三、组件与容器

### 一个进程里放多个节点

Test1 里节点编成了组件（component），但 launch 里是用 `Node` 单独启动的，一个节点一个进程。

本题把五个节点（四个仿真节点加 locator）放进**同一个容器进程**。好处有两个：

- **进程内通信**：同一进程内的节点之间传消息，可以直接传指针，不用序列化再反序列化。开启方式是给每个组件加 `use_intra_process_comms: True`
- **共享进程内的状态**：同一进程的节点可以共享静态变量。这一点对本题是硬性要求，见下

### 仿真节点必须在同一个进程里

四个仿真节点各自用自己的定时器发数据，但它们必须对"仿真从哪一刻开始"达成一致，否则算出来的真值属于不同的世界。`locator_sim/include/locator_sim/sim_clock.hpp` 用一个进程内的静态变量记录仿真起始时刻，谁先运行谁初始化。

这个办法只在同一进程里有效。拆成多个进程，每个进程各有一份静态变量，各自记录自己的起始时刻。

实测：四个仿真节点各起一个进程，**不报任何错误**，`verify_accuracy` 全部配对成功，但平均误差 35~39 mm，最大 70~83 mm，有零星的残余 roll 告警。

locator 自己放不放进同一个容器，对正确性没有影响（实测单独一个进程也能通过）。本题要求放进去，因为这样能用上进程内通信。

## 四、launch

### 在容器里启动组件

```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

ComposableNodeContainer(
    name='my_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    composable_node_descriptions=[
        ComposableNode(
            package='my_package',
            plugin='my_package::MyNode',       # RCLCPP_COMPONENTS_REGISTER_NODE 里写的类名
            name='my_node',
            parameters=[params_yaml],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        # 更多组件……
    ],
    output='both',
    emulate_tty=True,
)
```

`plugin` 填的是 C++ 类的完整名字，包括命名空间，和源文件末尾 `RCLCPP_COMPONENTS_REGISTER_NODE(...)` 里的一致。仿真四个节点的类名可以在 `locator_sim/CMakeLists.txt` 的 `PLUGIN` 里找到。

`locator_launcher/launch/sim_only.launch.py` 是一个可以直接运行的完整例子，只启动仿真不启动 locator。你要写的 `locator_launcher.launch.py` 在它的基础上扩展。

### 多个参数文件

`parameters` 是一个列表，可以放多个 yaml 文件和字典。**后面的覆盖前面的同名参数。**

```python
parameters=[base_yaml, override_yaml]
```

`base_yaml` 里定义了 `a: 1, b: 2`，`override_yaml` 里只定义了 `b: 3`，节点最终拿到 `a: 1, b: 3`。

本题要求 launch 提供一个 `override_params` 参数，指向一个额外的 yaml，叠加在所有节点的默认参数之后。默认值指向 `locator_launcher/config/empty.yaml`，它是一个合法但不覆盖任何参数的空文件。

这个参数有两个用途：

- 做实验时，不改默认配置就能换一组参数跑，比如提高串口频率看精度怎么变
- **批改时会用它传入另一组安装参数复测你的代码**，所以必须实现，而且必须作用到仿真和 locator 两侧

yaml 里用 `/**:` 作为顶层键，表示对所有节点生效。

### 命令行参数

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

port = LaunchConfiguration('port')

return LaunchDescription([
    DeclareLaunchArgument('port', default_value='8765', description='WebSocket port'),
    # 后面用 port 代替写死的值
])
```

启动时覆盖：

```bash
ros2 launch my_package my.launch.py port:=9000
```

`LaunchConfiguration` 在 launch 运行时才被替换成实际的值，替换出来的是字符串。把它放进 `parameters` 时，launch 会按 yaml 的规则自己推断类型，`'8765'` 会被当成整数。

推断有时会和你想要的不一样，比如一个本该是字符串的话题名恰好全是数字。想把类型写死，用 `ParameterValue`：

```python
from launch_ros.parameter_descriptions import ParameterValue

parameters=[{'port': ParameterValue(port, value_type=int)}]
```

### 用一个 launch 参数指定额外参数文件

本题要求 launch 提供 `override_params`：一个从命令行传入的参数文件路径，叠加在默认参数之后。做法是把前面两节合起来——把 `LaunchConfiguration` 当成一个参数文件路径，放进 `parameters` 列表：

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    base_yaml = os.path.join(
        get_package_share_directory('my_package'), 'config', 'params.yaml'
    )
    override = LaunchConfiguration('override_params')

    return LaunchDescription([
        DeclareLaunchArgument(
            'override_params',
            default_value=os.path.join(
                get_package_share_directory('my_package'), 'config', 'empty.yaml'
            ),
            description='extra YAML overriding any parameter',
        ),
        ComposableNodeContainer(
            # ...
            composable_node_descriptions=[
                ComposableNode(
                    # ...
                    parameters=[base_yaml, override],   # 后面的覆盖前面的
                ),
            ],
        ),
    ])
```

启动时换一个文件：

```bash
ros2 launch my_package my.launch.py override_params:=/path/to/my.yaml
```

`override` 要加进**每一个**需要被覆盖的组件，不能只加一个。本题仿真和 locator 两边都要加，否则两边拿到的安装量不一致，结果全错。

### 条件启动

```python
from launch.conditions import IfCondition

Node(
    package='foxglove_bridge',
    executable='foxglove_bridge',
    condition=IfCondition(LaunchConfiguration('use_foxglove')),
)
```

`use_foxglove:=false` 时这个节点不启动。

### 参数文件的路径

和 Test1 一样，用 `get_package_share_directory` 拿 share 目录再拼路径，不要写相对路径或绝对路径。

```python
os.path.join(get_package_share_directory('locator_sim'), 'config', 'sim_params.yaml')
```

## 五、运行时查看参数

```bash
ros2 param list /locator
ros2 param get /locator mount.camera_mount_pitch
```

改了 yaml 或者传了 `override_params` 之后，用这两条确认节点实际拿到的值。yaml 里写的嵌套结构，在节点里是用点号连接的名字，比如 `mount.camera_mount_pitch`。

## 六、本题怎么用

| 本文的哪一节 | 对应的 TODO |
|---|---|
| 一、QoS | 构造函数里两个订阅的 QoS |
| 二、回调组 | 构造函数里的回调组和 `SubscriptionOptions` |
| 二、容器的执行器；三、组件与容器 | launch 里容器的选择、哪些节点放进去 |
| 四、launch | `locator_launcher.launch.py` 全部 |

launch 的具体要求见 `src/locator_launcher/launch/README.md`。

## 七、排查

### 回调一次都不触发

```bash
ros2 node info /locator               # 订阅的话题名对吗？
ros2 topic info -v /话题名             # 两端 QoS 兼容吗？
```

然后回头翻启动日志，找 `incompatible QoS`。

### 节点起不来

组件在容器里加载失败时，容器进程本身不会退出，launch 也不会停，只打一条 ERROR。要往上翻日志找。

- `Component constructor threw an exception: Statically typed parameter 'mount.gimbal_yaw_offset_z' must be initialized.`：这个组件的 `parameters` 里没有传 yaml，或者传错了文件，或者参数名拼错了
- `Failed to find class with the requested plugin name 'locator::LocatorNod' in the loaded library`：`plugin` 写错了，和源文件末尾的注册宏对照

### 跑了一段时间后不再输出

先确认是不是锁死：

```bash
ros2 topic hz /tf                     # 云台 TF 还在更新吗？
```

本题正常运行时约 700 Hz。如果掉到 200 Hz 左右，说明只剩底盘在发，云台 TF 停了。回到 `Part_of_TF.md` 第五节，检查有没有阻塞等待。

### 没有报错，但误差几十毫米

检查仿真的四个节点是不是在同一个容器里：

```bash
ros2 node list
ps aux | grep component_container
```

---

<div align="right">—— 浙江大学 RoboMaster 算法组</div>
