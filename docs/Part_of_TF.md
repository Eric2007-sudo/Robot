<h1 align="center">Part of TF</h1>

这份文档讲 ROS 2 的坐标变换系统 tf2：坐标系怎样组成一棵树、变换怎样发布和查询，以及本题最关键的部分——**按哪个时间查**。

读完之后，你应该能写出 `tf_utils.hpp` 里的 `try_lookup_tf` 和 `to_transform_stamped`、`LocatorNode` 里的 `publish_static_transforms`，并且知道两个回调里的 TF 查询应该怎样写。

数学部分（旋转矩阵、复合、求逆）见 `Part_of_Robotics_Math.md`，本文沿用那里第一节的记号：`T_{A←B}` 把 B 系下的点变到 A 系。

## 先读这个

> **[07-ROS2 基础](https://hello-world-vision.github.io/Vision_Website/induction-training/07-ros2基础/)** 里的 tf2 一节
>
> 那里讲了 tf2 的基本概念和最小示例。本文假设你已经知道"TF 是什么、为什么需要它"，重点放在本题会踩到的地方。

## 一、坐标系树

### 本题的 TF 树

```
world（惯性系）
└── base_link（车体；virtual_serial 发布，动态，200 Hz）
    └── gimbal_yaw（云台 yaw 连杆；locator 发布，动态，跟 IMU 同频 500 Hz）
        └── gimbal_pitch（云台 pitch 连杆；locator 发布，动态，500 Hz）
            └── camera_link（相机机械系；locator 发布，静态）
                └── camera_optical（相机光学系；locator 发布，静态）
```

上面三段由仿真和 locator 在运行时不断更新，下面两段是物理固连的，启动时发一次就行。你要写的是 locator 负责的四段。

各坐标系的轴向：

![本题的坐标系与轴向](assets/frames_overview.png)

这些坐标系各自为什么存在、电机坐标系和固连量是怎么回事，见 `Part_of_Frames.md`。

### 树的规则

tf2 要求坐标系组成一棵**树**：

- 每个坐标系最多只有一个父坐标系
- 不能有环
- 整棵树只有一个根

查询两个坐标系之间的变换时，tf2 沿树找到它们的公共祖先，把路上每一段变换乘起来。你不需要自己做连乘，但每一段都必须有人发布，而且父子关系不能写反。父子写反之后，树会从中间断开：

```
world → base_link     （仿真发布，正确）
gimbal_yaw → base_link （父子写反了）
```

这时 base_link 有了两个父坐标系，tf2 不接受。查询会报找不到连接。

## 二、一条 TF 消息

```
geometry_msgs/msg/TransformStamped
  header.stamp        这个变换对应的时刻
  header.frame_id     父坐标系
  child_frame_id      子坐标系
  transform
    translation       子坐标系原点在父坐标系下的位置
    rotation          子坐标系相对父坐标系的姿态，四元数
```

一条消息描述的是 `T_{父←子}`：它把子坐标系下的点变到父坐标系，同时也是子坐标系在父坐标系里的位姿。

`to_transform_stamped(tf, parent_frame, child_frame, stamp)` 要做的就是把这四样东西填进去。`tf2::Transform` 转成消息里的 `transform` 字段，用 `tf2_geometry_msgs` 提供的 `tf2::toMsg`。

### 发布

```cpp
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

// 构造函数里
tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

// 发一条
tf_broadcaster_->sendTransform(msg);

// 一次发多条，类型是 std::vector<TransformStamped>
tf_broadcaster_->sendTransform(msgs);
```

动态变换发到 `/tf`，静态变换发到 `/tf_static`。

### 静态变换必须用静态广播器

静态变换只发一次。如果用普通的 `TransformBroadcaster` 发，后启动的节点、或者启动时还没准备好的订阅者，就永远收不到这一条，tf2 里也就不存在这一段。

`StaticTransformBroadcaster` 发布时用的 QoS 是 `transient_local`：发布者会保留最后一次发出的消息，之后才加入的订阅者也能收到。这也是"静态"两个字的实际含义。QoS 的细节见 `Part_of_ROS2.md` 第一节。

实测：把本题的两段静态变换改用普通广播器发，`verify_accuracy` 一个样本都配不上，日志持续报

```
tf chain world -> camera_optical not available at observation stamp, dropping
```

还有一点：`StaticTransformBroadcaster` 内部维护一张按子坐标系索引的表，每次 `sendTransform` 都把新的变换合并进去（同一个子坐标系就替换，新的就追加），然后把整张表重新发一遍。本题两段静态变换的子坐标系不同，所以分两次发或者一次发两条，结果一样。

## 三、查询

### Buffer 和 Listener

```cpp
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);
```

`TransformListener` 订阅 `/tf` 和 `/tf_static`，把收到的每一条变换连同时间戳存进 `Buffer`。Buffer 默认保留最近 10 秒的历史。之后的所有查询都是查 Buffer，不会再去网络上要数据。

第二个参数要的是**节点指针**，所以传 `this`（不是 `*this`）。第一个参数要 Buffer 的引用，所以要 `*tf_buffer_`。为什么两个参数一个加 `*` 一个不加，见 `Part_of_Cpp.md` 第一节。

这两行在骨架里已经写好了。

### lookupTransform

```cpp
geometry_msgs::msg::TransformStamped msg =
    tf_buffer_->lookupTransform(target_frame, source_frame, time);
```

返回 `T_{target←source}`：它把 source 系下的点变到 target 系。**第一个参数是输出坐标系。** 这个顺序和你的直觉可能相反，实测写反之后定位误差是 9.7 m，而且没有任何告警，因为反方向的变换同样存在、同样能查到。

返回值是消息类型，要转成 `tf2::Transform` 才能做乘法：

```cpp
tf2::Transform tf;
tf2::fromMsg(msg.transform, tf);
```

查不到时 `lookupTransform` **抛异常**，类型都是 `tf2::TransformException` 的子类。实际的报错文字：

| 情况 | 异常类型 | 报错文字 |
|---|---|---|
| 查询时刻比最新的数据还新 | `ExtrapolationException` | `Lookup would require extrapolation into the future.  Requested time 12.000000 but the latest data is at time 11.000000, ...` |
| 查询时刻比最早的数据还旧 | `ExtrapolationException` | `Lookup would require extrapolation into the past.  Requested time 5.000000 but the earliest data is at time 10.000000, ...` |
| 坐标系不存在 | `LookupException` | `"gimbal_yaw" passed to lookupTransform argument source_frame does not exist.` |
| 两个坐标系不在同一棵树上 | `ConnectivityException` | `Could not find a connection between 'world' and 'camera_optical' because they are not part of the same tree.` |

`try_lookup_tf` 要做的就是把这次查询包在 `try` 里：成功就把结果转成 `tf2::Transform` 写进输出参数、返回 `true`；失败就 `catch` 这个异常，把 `ex.what()` 交给 `err_info_handler`，返回 `false`。try / catch 的写法见 `Part_of_Cpp.md`。

### canTransform

```cpp
bool ok = tf_buffer_->canTransform(target_frame, source_frame, time);
```

只问能不能查到，不返回结果，不抛异常。参数顺序和 `lookupTransform` 一致。

## 四、按哪个时间查

这一节是本文的重点，也是本题最容易出错、出错后最难发现的地方。

### 每个变换都带时间戳

底盘在动，云台在转，`world → base_link` 这一段每 5 ms 就变一次。Buffer 里存的是一串带时间戳的变换。查询时必须说"我要哪个时刻的"。

### 三种时间，只有一种是对的

```cpp
// ① 消息自带的时间戳
const rclcpp::Time stamp = msg->header.stamp;

// ② 现在
const rclcpp::Time stamp = get_clock()->now();

// ③ 最新的一条（时间填 0）
const rclcpp::Time stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
```

只有 ① 是对的。原因要从数据是怎么来的说起。

### 时间线

本题的三路数据都有延迟，而且时间戳记的是**采样时刻**，不是发布时刻：

| 数据 | 频率 | 延迟 | 时间戳含义 |
|---|---|---|---|
| 底盘位姿（串口，`world → base_link`） | 200 Hz | 2 ms | 下位机采样时刻 |
| IMU 姿态 | 500 Hz | 10 ms | IMU 采样时刻 |
| 目标观测（detector） | 60 Hz | 20 ms | 相机曝光时刻 |

以 IMU 为例，设某次采样发生在时刻 `t_s`：

```
时间 →

t_s                    IMU 采样，云台此刻的姿态是 R(t_s)
t_s + 10 ms            IMU 消息发出，header.stamp = t_s
                       locator 的 imu_callback 被调用

此时 Buffer 里底盘位姿的最新一条，时间戳在 t_s + 3 ms 到 t_s + 8 ms 之间
（串口每 5 ms 一帧，延迟 2 ms）
```

IMU 测到的是 `t_s` 时刻的云台姿态。要从它减去底盘姿态，必须用**同一时刻** `t_s` 的底盘姿态。Buffer 里有 `t_s` 前后两帧的底盘数据，tf2 会在两帧之间线性插值，得到 `t_s` 那一刻的值。

用 ② `now()` 会怎样？`now()` 是 `t_s + 10 ms`，比 Buffer 里最新的底盘数据还新，查询会报 "extrapolation into the future"。实测：每一帧都查不到，全部丢帧，`verify_accuracy` 一个样本都配不上。

用 ③ 查最新会怎样？查到的是 `t_s + 3 ms` 到 `t_s + 8 ms` 之间某一刻的底盘姿态，和 IMU 采样时刻差了几毫秒。底盘的颠簸角频率在 7.5~11 rad/s 之间，几毫秒里姿态会变一点。target 回调更明显：观测是 20 ms 前曝光的，查最新查到的是比曝光时刻晚十几毫秒的云台和底盘。实测两个回调都查最新时：**全部配对成功，平均误差 45 mm，最大 117 mm，只有零星的残余 roll 告警。**

这是本题最隐蔽的错误。它不丢帧，几乎不报警，Foxglove 里画出来的轨迹看起来基本正常。只有拿真值逐点比对才能发现。

target 回调同理。观测的时间戳是曝光时刻，整条链要按曝光时刻查。

### 输出的时间戳

你发出去的东西，时间戳也要和输入一致：

- `imu_callback` 发布的两段云台 TF，`header.stamp` 用 IMU 消息的时间戳
- `target_callback` 发布的世界系目标，`header.stamp` 用观测消息的时间戳

`verify_accuracy` 和 `verify_gimbal` 都是按时间戳把你的输出和真值配对的。时间戳不对，它们配不上任何样本。更重要的是，下游节点拿到你的输出之后，也要按这个时间戳去做它自己的对齐。

### 延迟为什么设计成这样

你可能注意到 IMU 的延迟（10 ms）比串口的延迟加周期（2 + 5 = 7 ms）还长。这不是巧合。tf2 插值需要查询时刻**前后各有一帧**数据。IMU 回调被调用时，如果底盘数据还没覆盖到 `t_s` 之后，插值就做不了。

detector 的 20 ms 延迟也一样：观测发出时，云台和底盘在曝光时刻前后的 TF 都已经到齐了。

实测改小延迟会怎样：

| 改动 | 结果 |
|---|---|
| IMU 延迟 10 ms → 5 ms（低于 7 ms） | 仍能通过，但持续报 chassis pose not available，丢帧变多，误差变大 |
| IMU 延迟 → 0 | 全部丢帧 |
| detector 延迟 → 0 | 全部丢帧 |

这些延迟是仿真的参数，你不需要改。但它解释了为什么查询要按消息时间戳，而且查不到时要能接受。

## 五、不能阻塞

### 查不到怎么办

即使时间戳全对，偶尔也会查不到：启动的前几毫秒数据还没到齐，或者某一帧串口数据晚到了。

`lookupTransform` 和 `canTransform` 都有带超时参数的版本：

```cpp
tf_buffer_->lookupTransform(target, source, time, rclcpp::Duration::from_seconds(0.1));
tf_buffer_->canTransform(target, source, time, rclcpp::Duration::from_seconds(0.1));
```

数据没到时，它们会**阻塞等待**，直到数据到达或超时。看起来很合理：等一会儿就有了。

本题不能这样做。IMU 每 2 ms 来一帧，回调里等 100 ms，就会有 50 帧 IMU 在排队。排队期间云台 TF 不更新，target 回调查不到云台 TF，也开始等。等的时间越长，积压越多，后面的数据越过期，越查不到。这是一个正反馈，最后整条链路锁死。

实测：在单线程执行器下用 100 ms 超时，60 秒的运行里 3 次全部锁死，一个样本都没有。锁死和执行器、回调组的关系见 `Part_of_ROS2.md` 第二节。

### 正确做法：先探测，没有就丢

```cpp
if (!tf_buffer_->canTransform(target, source, stamp)) {
    // 数据还没到，丢掉这一帧
    return;
}
// 走到这里说明数据已经到了，查询不会失败、也不会等
```

不带超时参数的 `canTransform` 立刻返回。IMU 下一帧 2 ms 后就到，丢一帧完全没有影响，丢帧比积压好得多。

`try_lookup_tf` 故意不提供超时参数，就是为了防止有人"顺手"加上等待。单元测试 `TryLookupTf.DoesNotBlockWhenDataMissing` 会检查它在数据缺失时是否在 50 ms 内返回。

丢帧的时候打一条日志，用 `RCLCPP_WARN_THROTTLE` 限制频率，比如每 2 秒最多一条。500 Hz 的回调里不限频地打日志，日志本身就会把 CPU 占满。

## 六、REP-103 与相机光学系

### 机体系：x 前、y 左、z 上

ROS 的约定 REP-103 规定，机器人的机体坐标系 x 朝前、y 朝左、z 朝上，三个轴构成右手系。本题的 world、base_link、gimbal_yaw、gimbal_pitch、camera_link 都遵循这个约定。

### 光学系：x 右、y 下、z 前

相机的光学坐标系不一样。图像处理的惯例是：

- x 沿图像的行，向右
- y 沿图像的列，向下
- z 沿光轴，向前

OpenCV、PnP、深度相机的输出都用这个约定。REP-103 也专门为它留了位置，名字一般以 `_optical` 结尾。

本题 detector 发出的目标位姿就在 `camera_optical` 下。`camera_link` 和 `camera_optical` 原点重合，只差一个旋转，这个旋转就是 `camera_optical_rotation`。

回头看第一节那张图：`camera_optical` 的 z 轴（光轴）画的方向，和 `camera_link` 的 x 轴（机械系的前方）是一致的。这就是这两套轴向的对应关系的一部分。

### 怎样写出这个旋转

按 `Part_of_Robotics_Math.md` 第二节的规则：`R_{link←optical}` 的三列，分别是光学系的 x、y、z 轴在 camera_link 下的方向。

所以只需要回答三个问题：

1. 光学系的 x 轴（像面向右）指向 camera_link 的哪个方向？
2. 光学系的 y 轴（像面向下）指向 camera_link 的哪个方向？
3. 光学系的 z 轴（光轴向前）指向 camera_link 的哪个方向？

camera_link 是 x 前 y 左 z 上。把三个答案写成三个列向量，按列排好，再按行抄进 `tf2::Matrix3x3` 的构造函数。

写完用单元测试检查。`CameraOpticalRotation` 下的几个测试分别检查：行列式是否为 +1、光轴正前方的点是否到了正前方、像面右侧的点是否到了车的右侧、像面下方的点是否到了下方。

实测这里写错的几种情况：

| 写法 | 定位误差 |
|---|---|
| 漏掉这个旋转（单位阵） | 18 m |
| 按行填了轴向（等于转置） | 18 m |
| 一根轴符号错（变成左手系） | 18 m |
| 两根轴符号错（仍是右手系，等于多转了 180°） | 平均 263 mm |

最后一行的误差量级和时间问题接近，不能只凭误差大小判断错在哪。

## 七、命令行工具

### tf2_echo：看两个坐标系之间的变换

```bash
ros2 run tf2_ros tf2_echo world base_link
```

输出是 `T_{world←base}`，也就是 base_link 在 world 里的位姿，和 `lookupTransform("world", "base_link", ...)` 的结果相同。第一个参数是输出坐标系。

注意 `tf2_echo` 自己的帮助信息把第一个参数叫 `source_frame`，和 `lookupTransform` 里 source 的含义正好相反，这是 tf2_echo 命名上的历史问题。它的帮助里另有一句 "This is the transform to get data from target_frame into the source_frame"，说的就是这个意思。记行为就好：`tf2_echo A B` 打印的是 B 在 A 里的位姿。

一次实际的输出：

```
At time 1791031633.698919806
- Translation: [-7.345, 9.490, 0.274]
- Rotation: in Quaternion (xyzw) [0.035, 0.018, 0.945, -0.323]
- Rotation: in RPY (radian) [0.012, -0.079, -2.483]
- Rotation: in RPY (degree) [0.690, -4.528, -142.260]
- Matrix:
 -0.788  0.613  0.055 -7.345
 -0.610 -0.790  0.058  9.490
  0.079  0.012  0.997  0.274
  0.000  0.000  0.000  1.000
```

`Matrix` 是 4×4 齐次变换，左上 3×3 是旋转，最右一列是平移。旋转矩阵的列就是 base_link 的三条轴在 world 下的方向，可以直接拿来检查轴向对不对。

刚启动时会先打一行 `Invalid frame ID ... frame does not exist`，那是 tf2_echo 自己的 Buffer 还没收到数据，等一两秒就好。

静态变换的时间戳显示为 `At time 0.0`，这是正常的。静态变换对任何时刻都有效。

### view_frames：画出整棵树

```bash
ros2 run tf2_tools view_frames
```

收集 5 秒的 TF 数据，在当前目录生成一个 PDF，画出整棵树，每一段标着发布频率和最新时间戳。树断了、父子写反了，一眼就能看出来。

### 看原始消息

```bash
ros2 topic echo /tf_static --once
ros2 topic hz /tf
```

`/tf_static` 能看到静态变换的 `frame_id`、`child_frame_id` 和数值。`/tf` 的频率在本题正常运行时约 700 Hz：底盘 200 Hz 加云台 500 Hz（两段放在同一条消息里发）。

## 八、本题怎么用

| 本文的哪一节 | 对应的 TODO |
|---|---|
| 二、TF 消息 | `tf_utils::to_transform_stamped` |
| 三、lookupTransform；五、不能阻塞 | `tf_utils::try_lookup_tf` |
| 二、静态广播器 | `LocatorNode::publish_static_transforms` |
| 六、光学系 | `tf_utils::camera_optical_rotation` |
| 四、时间；五、先探测后查询 | `imu_callback`、`target_callback` 里的查询和发布 |

两个回调里要回答的问题：

- 查哪两个坐标系之间的变换？target 是谁，source 是谁？
- 按哪个时间查？
- 查不到时怎么处理？
- 发出去的消息，`frame_id`、`child_frame_id`、时间戳分别填什么？

target 回调里，链路的起点用消息自带的 `header.frame_id`，不要写死 `"camera_optical"`。

## 九、排查

先跑三个校验脚本，按结果对照下表：

```bash
ros2 run locator_launcher verify_gimbal.py      # 云台关节角
ros2 run locator_launcher verify_accuracy.py    # 世界系目标位置
ros2 run locator_launcher verify_cassini.py     # 轨迹形状
```

### 一个样本都配不上

链路断了，或者时间对不上。

```bash
ros2 run tf2_tools view_frames           # 树完整吗？父子方向对吗？
ros2 topic echo /tf_static --once        # 静态变换发出来了吗？
```

然后看 locator 的日志：

- 一直报 `tf chain world -> camera_optical not available`：链路上缺了某一段。用 view_frames 找是哪段
- 一直报 `chassis pose not available at imu stamp`：IMU 回调查底盘位姿时查不到。检查查询用的是不是消息时间戳
- verify_gimbal 提示"看到了反向的 TF"：云台 TF 的父子写反了

### 全部配上，误差是米级

链路通了，几何算错了。

- 先看 `verify_gimbal`。关节角错了，回到 `Part_of_Robotics_Math.md` 第七节检查反解公式
- 关节角对了，就是静态变换或者 target 回调的问题。用单元测试检查 `camera_optical_rotation`，用 `tf2_echo gimbal_pitch camera_link` 检查安装量

### 全部配上，误差几十到上百毫米

几乎都是时间问题：

- 查询用的不是消息时间戳
- 仿真节点没有放在同一个进程里（见 `Part_of_ROS2.md` 第三节）

如果这两项都排除了，再检查光学系矩阵是不是有两根轴的符号同时错了。

---

<div align="right">—— 浙江大学 RoboMaster 算法组</div>
