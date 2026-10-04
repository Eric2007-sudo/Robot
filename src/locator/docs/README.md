# locator 节点的接口契约

> 任务说明在仓库根目录的 `QUESTION.md`，教学文档在 `docs/`。本文是查阅用的技术资料：
> 这个节点吃什么、吐什么、各字段什么含义。写代码时对照这里。

## 输入

| 话题 | 类型 | QoS | `header.frame_id` | `header.stamp` |
|---|---|---|---|---|
| `locator_sim/imu/data` | `sensor_msgs/msg/Imu` | `SensorDataQoS().keep_last(50)`（best_effort） | `gimbal_pitch` | IMU 采样时刻 |
| `locator_sim/detector/target_pose` | `geometry_msgs/msg/PoseStamped` | `QoS(10)`（reliable） | `camera_optical` | 相机曝光时刻 |
| `/tf` | `tf2_msgs/msg/TFMessage` | tf2 默认 | — | 底盘位姿的下位机采样时刻 |

### IMU 消息

- **`orientation`**：pitch 连杆相对 `world` 的**绝对姿态**，单位四元数。不是关节角。
  IMU 固连在云台的 pitch 连杆上
- **`angular_velocity`**：陀螺原始量，表达在**机体系**下，不是欧拉角速率
- **`linear_acceleration`**：加速度计测的是**比力**，含重力反作用。静止时读数约 9.81，
  不是 0

本题只用到 `orientation`。另外两个字段保留真实语义，但不需要处理。

### 目标观测

`PoseStamped`，`pose` 是目标在 `camera_optical` 下的位姿。`position` 是位置，
`orientation` 是目标自身的姿态（本任务不用于定位，只跟随位置）。

detector 有视场裁剪：目标不在视场内时不发消息。下游要能容忍数据间断。

## 输出

| 话题 | 类型 | 说明 |
|---|---|---|
| `/tf` | `tf2_msgs/msg/TFMessage` | `base_link → gimbal_yaw`、`gimbal_yaw → gimbal_pitch` 两段动态变换 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `gimbal_pitch → camera_link`、`camera_link → camera_optical` 两段静态变换 |
| `locator/target_in_world` | `geometry_msgs/msg/PoseStamped` | 目标在 `world` 下的位姿 |
| `locator/visualized_marker` | `visualization_msgs/msg/Marker` | 可视化，代码已给 |

输出消息的时间戳必须与对应输入的**时间戳相同**：

- 两段云台 TF 的 `header.stamp` = IMU 消息的 `header.stamp`
- `target_in_world` 的 `header.stamp` = 观测消息的 `header.stamp`

校验脚本按时间戳把输出和真值逐点配对，时间戳不一致就配不上。

## 坐标系

```
world                       惯性系，REP-103：x 前、y 左、z 上
└── base_link               车体，虚拟串口发布（含颠簸）
    └── gimbal_yaw          云台 yaw 关节，绕 base_link 的 +z 转
        └── gimbal_pitch    云台 pitch 关节，绕 gimbal_yaw 的 +y 转，IMU 固连在此
            └── camera_link        相机机械系，REP-103
                └── camera_optical 相机光学系，OpenCV 约定：x 右、y 下、z 前
```

名字定义在 `locator_common/include/locator_common/frames.hpp`，代码里用 `frames::WORLD`
这样的常量，不要写字符串。

## 参数

`config/locator_params.yaml`：

| 参数 | 含义 |
|---|---|
| `mount.gimbal_yaw_offset_*` | yaw 关节原点在 `base_link` 下的位置（m） |
| `mount.gimbal_pitch_offset_*` | pitch 关节原点在 `gimbal_yaw` 下的位置（m） |
| `mount.camera_offset_*` | 相机在 `gimbal_pitch` 下的位置（m） |
| `mount.camera_mount_roll/pitch/yaw` | 相机相对 pitch 连杆的安装倾角（rad） |
| `enable_publish_marker` | 是否发布可视化 marker |
| `imu_topic` / `target_topic` | 订阅话题名 |
| `world_target_topic` / `marker_topic` | 发布话题名 |

`mount.*` 是物理固连量，必须与 `locator_sim/config/sim_params.yaml` 里的 `scene.*`
对应项一致。批改时会用 `override_params` 同时改两边。

## 真值旁路

仿真另外发了几路真值，**locator 不订阅**，只给校验脚本用：

| 话题 | 内容 |
|---|---|
| `locator_sim/detector/target_truth` | 目标在 `world` 下的真值位姿，与观测同时间戳 |
| `locator_sim/imu/gimbal_joint_truth` | 云台两个关节角的真值，与 IMU 消息同时间戳 |

`verify_accuracy.py` 用前者，`verify_gimbal.py` 用后者。它们的存在是为了把定位误差
拆开：关节角错了就查 imu 回调，关节角对了但位置错了就查静态 TF 或 target 回调。
