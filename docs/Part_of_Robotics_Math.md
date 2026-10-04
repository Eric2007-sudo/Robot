<h1 align="center">Part of Robotics and Math</h1>

这份文档讲本周任务需要的那部分机器人学数学：旋转矩阵、坐标变换的复合与求逆、欧拉角，以及怎样从 IMU 测到的绝对姿态反解出关节角。

读完之后，你应该能写出 `tf_utils.hpp` 里的 `make_transform` 和 `camera_optical_rotation`，并且能推出 `imu_callback` 里反解关节角的那一行公式。

文档用到的记号在第一节定下，后面 TF、ROS2 两份文档沿用同一套。如果读到别处的记号对不上，以这里为准。

## 一、记号

坐标变换最容易错的地方不在计算，而在**方向**。同一个矩阵，"A 到 B"和"B 到 A"两种叫法都有人用，本周任务里又同时出现数学推导、代码变量名、tf2 的函数参数和 TF 消息字段四种写法。先定一套规则：

```
R_{A←B}    把 B 系下表达的向量，变成 A 系下的表达
T_{A←B}    同上，带平移的齐次变换
```

箭头从右往左读：右边是输入的坐标系，左边是输出的坐标系。

例如 `T_{world←base}` 作用在一个 base_link 系下的点上，得到同一个点在 world 系下的坐标。它同时也描述了 base_link 在 world 里的位姿：平移部分是 base_link 原点在 world 下的位置，旋转部分是 base_link 的朝向。

这套记号和代码里的四种写法一一对应：

| 数学记号 | 代码变量名（本项目的命名习惯） | tf2 查询 | TF 消息字段 |
|---|---|---|---|
| `T_{world←base}` | `chassis_to_world` | `lookupTransform("world", "base_link", t)` | `frame_id = "world"`，`child_frame_id = "base_link"` |
| `T_{optical←target}` | `target_to_optical` | — | detector 发来的 `PoseStamped`，`frame_id = "camera_optical"` |

记住三条：

- 变量名 `X_to_Y` 就是 `T_{Y←X}`，把 X 系下的点变到 Y 系
- `lookupTransform(target, source, t)` 返回 `T_{target←source}`，第一个参数是输出坐标系
- TF 消息里 `frame_id` 是父、`child_frame_id` 是子，消息描述的是 `T_{父←子}`

## 二、旋转矩阵

### 列向量就是子系的三条轴

`R_{A←B}` 是一个 3×3 矩阵。它的三列分别是 B 系的 x、y、z 轴单位向量**在 A 系下的坐标**：

```
R_{A←B} = [ x_B 在 A 下 | y_B 在 A 下 | z_B 在 A 下 ]
```

这一条是本文最重要的结论，后面填矩阵、检查矩阵都靠它。

例：B 系绕 A 系的 z 轴转了 90°。B 的 x 轴指向 A 的 +y，B 的 y 轴指向 A 的 -x，z 轴不变。于是

```
R_{A←B} = ⎡ 0  -1   0 ⎤
          ⎢ 1   0   0 ⎥
          ⎣ 0   0   1 ⎦
```

第一列 `(0, 1, 0)` 就是"B 的 x 轴在 A 下是 +y"。

反过来用这条规则，**看到两个坐标系的轴怎么对应，就能直接写出矩阵**，不需要分解成几次绕轴旋转。本题的 `camera_optical_rotation` 就该这样写：先弄清楚光学系的 x、y、z 三条轴各自指向 camera_link 的哪个方向，再把这三个方向按列填进去。光学系的轴向约定见 `Part_of_Frames.md` 第二节和 `Part_of_TF.md` 第六节。

### 三个基本旋转

按右手定则，绕各轴转 a 角：

```
Rx(a) = ⎡ 1    0       0    ⎤
        ⎢ 0  cos a  -sin a  ⎥
        ⎣ 0  sin a   cos a  ⎦

Ry(a) = ⎡  cos a  0  sin a ⎤
        ⎢    0    1    0   ⎥
        ⎣ -sin a  0  cos a ⎦

Rz(a) = ⎡ cos a  -sin a  0 ⎤
        ⎢ sin a   cos a  0 ⎥
        ⎣   0       0    1 ⎦
```

注意 `Ry` 里 `sin a` 的位置和另外两个不同。这不是笔误，用右手定则可以验证：绕 +y 正转时，+x 轴转向 -z。

右手定则的用法：右手大拇指指向转轴正方向，四指弯曲的方向就是正转方向。本题里 pitch 绕 +y 正转会让云台低头，所以目标在上方时 pitch 是负的。

### 旋转矩阵的两个性质

旋转矩阵是**正交矩阵**，行列式为 +1：

```
R · Rᵀ = I
det(R) = +1
```

第一条说明它的逆就是转置，第二条说明它保持右手系。行列式为 -1 的矩阵是一个镜像，不是旋转。把某根轴的方向填反，行列式就会变成 -1。

## 三、复合与求逆

### 复合：相邻下标相消

```
R_{A←C} = R_{A←B} · R_{B←C}
```

读法：先把 C 下的向量变到 B，再从 B 变到 A。乘号两边相邻的下标（这里是 B）相同，可以"消掉"。下标对不上的乘法一定是写错了。

整条链也一样：

```
T_{world←optical} = T_{world←base} · T_{base←yaw} · T_{yaw←pitch} · T_{pitch←link} · T_{link←optical}
```

tf2 查询 `world → camera_optical` 时，内部做的就是这个连乘。

### 求逆：转置

```
R_{B←A} = R_{A←B}⁻¹ = R_{A←B}ᵀ
```

用这条可以从复合式里解出中间的一段。已知 `R_{A←C}` 和 `R_{A←B}`，求 `R_{B←C}`：

```
R_{A←C} = R_{A←B} · R_{B←C}
R_{A←B}ᵀ · R_{A←C} = R_{B←C}
```

两边同时**左乘** `R_{A←B}ᵀ`。左乘和右乘结果不同，矩阵乘法不满足交换律。这里乘在左边，是因为要消掉的 `R_{A←B}` 在等式右边的最左侧。

第七节的姿态反解就是这一步。

## 四、齐次变换

旋转加平移，合在一起写成 4×4 矩阵：

```
T_{A←B} = ⎡ R_{A←B}   t ⎤        T_{A←B} · p = R_{A←B} · p + t
          ⎣   0ᵀ      1 ⎦
```

`t` 是 B 的原点在 A 下的坐标。作用在点上时**先转再平移**。顺序反过来结果就错了：

```
R = Rz(90°), t = (2, 0, 0), p = (1, 0, 0)

R·p + t   = (2, 1, 0)      ← 正确
R·(p + t) = (0, 3, 0)      ← 错误
```

齐次变换的逆：

```
T_{B←A} = ⎡ Rᵀ   -Rᵀ·t ⎤
          ⎣ 0ᵀ      1  ⎦
```

平移部分是 `-Rᵀ·t`，不是 `-t`。只有 R 是单位阵时两者才相等。

在 tf2 里，齐次变换是 `tf2::Transform`：

```cpp
tf2::Transform T(rotation, translation);   // rotation 可以是 Quaternion 或 Matrix3x3
tf2::Vector3 p_A = T * p_B;                // 作用在点上
tf2::Transform T_AC = T_AB * T_BC;         // 复合
tf2::Transform T_BA = T_AB.inverse();      // 求逆
T.getBasis();                              // 旋转部分，Matrix3x3
T.getOrigin();                             // 平移部分，Vector3
```

`tf2::Matrix3x3` 的构造函数按**行**填：

```cpp
tf2::Matrix3x3 m(
    m00, m01, m02,   // 第 0 行
    m10, m11, m12,   // 第 1 行
    m20, m21, m22    // 第 2 行
);
```

而第二节说的规则是按**列**理解。所以从"三条轴各指向哪"写矩阵时，先在纸上把三列排好，再一行一行抄进构造函数。按行直接填轴向，得到的是转置，也就是反方向的旋转。

## 五、欧拉角

### tf2 用的约定

欧拉角是用三个角描述一个旋转。三个角绕哪三根轴、按什么顺序转，有十几种约定，不同的库各不相同。**本题只用 tf2 的约定**：

```cpp
tf2::Quaternion q;
q.setRPY(roll, pitch, yaw);
```

roll 绕 x、pitch 绕 y、yaw 绕 z。配合 REP-103 的轴向（x 前、y 左、z 上），三个角的物理含义是：

| 角 | 绕哪根轴 | 在车上是什么动作 | 本题里谁在动 |
|---|---|---|---|
| roll | x（车头方向） | 车身左右侧倾 | 底盘颠簸 |
| pitch | y（车身左侧） | 车头抬起或低下；云台俯仰 | 底盘颠簸 + pitch 电机 |
| yaw | z（车顶方向） | 车或云台左右转头 | 底盘转向 + yaw 电机 |

对照 `Part_of_Frames.md` 第一节那张图就能看出来：pitch 绕 y 轴，而图上 `gimbal_pitch_y` 正是画了旋转箭头的那根轴；yaw 绕 z 轴，对应 `gimbal_yaw` 竖直的那根。所以云台两个电机角在 RPY 里的位置是固定的：**yaw 电机角填 yaw 位，pitch 电机角填 pitch 位，roll 位始终是 0**（二轴云台没有 roll 关节）。

一个符号问题：按右手定则，绕 +y 正向转动时 +x 轴转向 -z，也就是**车头往下**。所以目标在高处时，pitch 电机角是负的。本题目标飞在 8 m 高空，解出来的 pitch 一直是负值，这是对的。

组合方式是：

```
R = Rz(yaw) · Ry(pitch) · Rx(roll)
```

这个式子有两种等价的理解：

- **固定轴**：绕固定不动的 x 轴转 roll，再绕固定的 y 轴转 pitch，最后绕固定的 z 轴转 yaw
- **动轴（内旋）**：先绕 z 转 yaw，再绕转过之后的新 y 轴转 pitch，最后绕新的 x 轴转 roll

两种理解得到同一个矩阵。tf2 的注释写的是 "fixed axis"，本题的云台用动轴理解更直观：先 yaw 关节转，pitch 关节装在 yaw 连杆上，绕的是已经转过的 y 轴。

`make_transform(translation, roll, pitch, yaw)` 要做的就是用 `setRPY` 拼出旋转，再和平移一起组成 `tf2::Transform`。

### 一个名字相近的陷阱

`tf2::Quaternion` 里还有一个 `setEuler(yaw, pitch, roll)`。名字和参数看起来都像，但它的注释写的是

```
@param yaw Angle around Y
@param pitch Angle around X
@param roll Angle around Z
```

绕的轴完全不同，是 Y-X-Z 约定。用它代替 `setRPY`，编译不报错，运行不报警，算出来的旋转是错的。单元测试里的 `MakeTransform.MatchesSetRPY` 就是为了挡住这种写法。

### 分解：从矩阵取回三个角

```cpp
tf2::Matrix3x3 m(q);
double yaw, pitch, roll;
m.getEulerYPR(yaw, pitch, roll);
```

`getEulerYPR` 按 `Rz(yaw)·Ry(pitch)·Rx(roll)` 分解，和 `setRPY` 互为逆运算。还有一个 `getRPY(roll, pitch, yaw)`，它内部就是调用 `getEulerYPR`，只是**参数顺序反过来**。两个函数都能用，但参数位置不能混：

```cpp
m.getEulerYPR(yaw, pitch, roll);   // 正确
m.getRPY(roll, pitch, yaw);        // 正确，同一个分解
m.getRPY(yaw, pitch, roll);        // 错：yaw 和 roll 互换了，编译照样能过
```

把 `Rz(y)·Ry(p)·Rx(r)` 乘开，看第三行：

```
R[2][0] = -sin p
R[2][1] =  cos p · sin r
R[2][2] =  cos p · cos r
```

所以 `pitch = -asin(R[2][0])`，`roll = atan2(R[2][1], R[2][2])`。yaw 从第一列取：`yaw = atan2(R[1][0], R[0][0])`。

### 万向锁

当 `pitch = ±90°` 时 `cos p = 0`，上面 roll 和 yaw 的公式分子分母同时为零，yaw 和 roll 无法区分：绕 z 转和绕 x 转产生的效果混在了一起。这叫万向锁。

tf2 在这个点上会把 yaw 定为 0，把全部转动算进 roll。接近 ±90° 时虽然还能算，但数值很敏感，小误差会被放大。

本题云台 pitch 的机械限位是 ±1.45 rad（约 83°），默认场景里目标最高时需要约 1.27 rad（约 73°），离奇异点还有距离，正常运行不会碰到。但如果你修改了场景参数，让目标几乎正好在头顶，就要知道分解会出问题。

## 六、四元数与欧拉角的互转

ROS 消息里的姿态一律是四元数：`Imu` 的 `orientation`、`TransformStamped` 的 `rotation`、`Pose` 的 `orientation` 都是。tf2 内部也用四元数存旋转。所以 IMU 的姿态到你手里时是四元数，而反解关节角要用矩阵或欧拉角，中间必须转一次。

本题不需要手算四元数，但**每个转换函数的参数顺序必须记准**。下面这张表是本节的重点，顺序写错编译不会报错，运行也不报警。

### 参数顺序对照表

| 做什么 | 怎么写 | 参数顺序 |
|---|---|---|
| 消息 → tf2 四元数 | `tf2::fromMsg(msg->orientation, q)` | 先消息，后目标 |
| tf2 四元数 → 消息 | `msg.rotation = tf2::toMsg(q)` | 返回值 |
| 四元数 → 矩阵 | `tf2::Matrix3x3 m(q)` | 构造函数直接收四元数 |
| 矩阵 → 四元数 | `m.getRotation(q)` | 结果写进 `q` |
| 欧拉角 → 四元数 | `q.setRPY(roll, pitch, yaw)` | **roll, pitch, yaw** |
| 矩阵 → 欧拉角 | `m.getEulerYPR(yaw, pitch, roll)` | **yaw, pitch, roll** |
| 矩阵 → 欧拉角（另一个） | `m.getRPY(roll, pitch, yaw)` | **roll, pitch, yaw** |
| 四元数 → 欧拉角（跳过矩阵） | `tf2::getEulerYPR(q, yaw, pitch, roll)` | **yaw, pitch, roll**，需要 `#include <tf2/utils.hpp>` |

最后一行的 `tf2::getEulerYPR` 是一个模板函数，消息、`tf2::Quaternion`、`tf2::Transform` 都能直接传进去，不用先手工转一次。同一个头文件里还有 `tf2::getYaw(a)`，只要 yaw 时用它更省事。

看第五、六、七行：**设置用 RPY 顺序，而两个分解函数一个是 YPR 顺序、一个是 RPY 顺序。** 这是 tf2 里最容易写错的地方。记法：函数名里字母的顺序就是参数的顺序。

- `setRPY` → R、P、Y → `(roll, pitch, yaw)`
- `getEulerYPR` → Y、P、R → `(yaw, pitch, roll)`
- `getRPY` → R、P、Y → `(roll, pitch, yaw)`

`getRPY` 内部就是调用 `getEulerYPR`，两者是同一个分解，只是参数位置不同。所以这三种写法里，只有最后一种是错的：

```cpp
m.getEulerYPR(yaw, pitch, roll);   // 对
m.getRPY(roll, pitch, yaw);        // 对，同一个结果
m.getRPY(yaw, pitch, roll);        // 错：yaw 和 roll 互换了，编译照样过
```

实测最后那种写法，定位误差 11.9 m，并且会触发残余 roll 告警。

### 四元数本身的几个易错

**构造函数的参数顺序是 `(x, y, z, w)`，w 在最后。** `geometry_msgs::msg::Quaternion` 的字段顺序也是 `x, y, z, w`。有些库（比如 Eigen）把 w 放在最前面，混用时要当心。

```cpp
tf2::Quaternion q(x, y, z, w);       // 四个分量，w 在最后
```

想从欧拉角建四元数，用 `setRPY`，不要去找现成的三参数构造：

```cpp
tf2::Quaternion q;
q.setRPY(roll, pitch, yaw);
```

（较新的发行版给 `tf2::Quaternion` 加了一个 `Quaternion(roll, pitch, yaw)` 三参数构造，但 Jazzy 上没有。写 `setRPY` 两个发行版都对。）

**默认构造的消息不是合法旋转。** `geometry_msgs::msg::Quaternion` 的默认值是 `x=0, y=0, z=0, w=1`，这是单位旋转，没问题。但如果你自己把四个分量都置零，那不是任何旋转，tf2 用到它会出错。表示"不旋转"要用 `w = 1`。

**`q` 和 `-q` 是同一个旋转。** 两个四元数，四个分量全部取反，描述的是完全相同的姿态。所以判断两个姿态是否一致不能逐分量比较，要比它们之间的夹角：

```cpp
q1.angleShortestPath(q2) < 1e-9       // 对
q1.x() == q2.x() && ...               // 错：q 和 -q 会被判成不同
```

单元测试 `MakeTransform.MatchesSetRPY` 用的就是前一种写法。

### 本题的用法

IMU 回调里，姿态从消息到可用的矩阵，走的是这条路：

```cpp
tf2::Quaternion imu_orientation;
tf2::fromMsg(msg->orientation, imu_orientation);   // 消息 → 四元数
tf2::Matrix3x3 R_imu(imu_orientation);             // 四元数 → 矩阵
```

拿到矩阵之后才能做第七节的复合求逆。反解出两个角之后，再用 `setRPY` 拼回变换发布出去——这一步由 `make_transform` 完成。

## 七、从绝对姿态反解关节角

这一节用一个和本题结构相同、但背景不同的例子，把反解完整走一遍。

### 场景

一个固定在无人机机臂上的二轴云台：

- `mount` 是机臂上的安装座。无人机飞行时会倾斜，安装座的姿态 `R_{world←mount}` 由飞控给出
- `pan` 关节绕 mount 的 z 轴转，`tilt` 关节绕 pan 连杆的 y 轴转
- IMU 固连在 tilt 连杆上，测的是 tilt 连杆**相对世界**的绝对姿态 `R_{world←tilt}`

要求：已知 `R_{world←mount}` 和 `R_{world←tilt}`，求两个关节角。

### 推导

IMU 测到的姿态是整条链的复合：

```
R_{world←tilt} = R_{world←mount} · R_{mount←tilt}
```

两边左乘 `R_{world←mount}ᵀ`，解出相对旋转：

```
R_{mount←tilt} = R_{world←mount}ᵀ · R_{world←tilt}
```

相对旋转由两个关节组成，先 pan 后 tilt：

```
R_{mount←tilt} = Rz(pan) · Ry(tilt)
```

这正是 `setRPY(0, tilt, pan)` 的形式，roll 为 0。所以用 `getEulerYPR` 分解相对旋转，yaw 位置得到 pan，pitch 位置得到 tilt，roll 位置理论上是 0。

为什么 roll 一定是 0？把 `Rz(y)·Ry(p)` 乘开：

```
Rz(y)·Ry(p) = ⎡ cos y·cos p   -sin y   cos y·sin p ⎤
              ⎢ sin y·cos p    cos y   sin y·sin p ⎥
              ⎣   -sin p         0       cos p     ⎦
```

`R[2][1]` 恒为 0，而 roll 的分子正是 `R[2][1]`。

### 一组数值

设机臂姿态是 yaw 30°、roll 10°（飞机侧倾），关节真值是 pan 60°、tilt -45°。

| 做法 | 分解出的 yaw / pitch / roll |
|---|---|
| `R_mountᵀ · R_imu` 再分解（正确） | 60.0° / -45.0° / 0.0° |
| 直接分解 `R_imu`（没减去安装座姿态） | 83.6° / -53.4° / 8.4° |
| `R_imu · R_mountᵀ` 再分解（乘反了） | 38.9° / -44.7° / 33.0° |

有两点值得看：

- 后两种错误写法分解出的 roll 都不为零。这就是 roll 可以当作自检信号的原因：理论上为零的量算出来不为零，说明前面某一步错了
- 如果机臂只有 yaw、没有倾斜，直接分解 `R_imu` 会得到 yaw = 30° + 60° = 90°，pitch 照样是 -45°，roll 也是 0。也就是说**底盘不颠簸的时候，错误写法的 pitch 碰巧是对的，roll 也不报警**。本题的底盘有 pitch 和 roll 两个方向的颠簸，就是为了让这种错误暴露出来

### 代码

```cpp
// R_{world←mount}：安装座姿态
const tf2::Matrix3x3 R_world_mount = mount_to_world.getBasis();
// R_{world←tilt}：IMU 测到的绝对姿态
const tf2::Matrix3x3 R_world_tilt(imu_orientation);

// R_{mount←tilt} = R_{world←mount}ᵀ · R_{world←tilt}
const tf2::Matrix3x3 R_mount_tilt = R_world_mount.transpose() * R_world_tilt;

double pan = 0.0, tilt = 0.0, roll = 0.0;
R_mount_tilt.getEulerYPR(pan, tilt, roll);
// roll 理论上为 0，|roll| 明显大于 0 时值得报警
```

这段代码只处理数学。实际节点里还要回答几个问题：安装座姿态从哪里来、按哪个时间查、查不到怎么办、算出来的关节角怎样发布出去。这些在 `Part_of_TF.md` 里讲。

## 八、自己检查一个旋转矩阵

写完一个旋转矩阵，不用等整条链跑起来，先做三项检查：

1. **正交**：`R · Rᵀ` 是否等于单位阵
2. **右手系**：`R.determinant()` 是否等于 +1。等于 -1 说明某根轴填反了
3. **代一个点**：挑一个物理含义明确的点算一下。比如"光轴正前方 2 m 的点"，在光学系下是 `(0, 0, 2)`，变到 camera_link 之后应当是正前方，也就是 `(2, 0, 0)`

单元测试 `test_tf_utils.cpp` 里的 `CameraOpticalRotation.*` 做的就是这三项检查。测试失败时读一下失败的是哪一条，比直接看误差数字更容易定位。

## 九、小练习

每题几分钟，做完再看答案。

**1.** B 系绕 A 系的 x 轴转了 90°，写出 `R_{A←B}`。

**2.** 已知 `R_{A←B} = Rz(90°)`，`t = (1, 0, 0)`。B 系下的点 `p = (0, 1, 0)` 在 A 系下是多少？

**3.** 下面哪个式子是对的？

```
(a) R_{A←C} = R_{B←C} · R_{A←B}
(b) R_{A←C} = R_{A←B} · R_{B←C}
(c) R_{B←C} = R_{A←C} · R_{A←B}ᵀ
(d) R_{B←C} = R_{A←B}ᵀ · R_{A←C}
```

**4.** 一个矩阵三列都是单位向量、两两正交，但 `det = -1`。它是旋转矩阵吗？

<details>
<summary>答案</summary>

**1.** B 的 x 轴还是 A 的 +x；B 的 y 轴转到 A 的 +z；B 的 z 轴转到 A 的 -y。按列填：

```
⎡ 1  0   0 ⎤
⎢ 0  0  -1 ⎥
⎣ 0  1   0 ⎦
```

和 `Rx(90°)` 一致。

**2.** 先转：`Rz(90°)·(0, 1, 0) = (-1, 0, 0)`。再平移：`(-1, 0, 0) + (1, 0, 0) = (0, 0, 0)`。这个点正好落在 A 的原点。

**3.** (b) 和 (d)。(a) 和 (c) 的相邻下标对不上。

**4.** 不是。它是一个旋转再加一次镜像，会把右手系变成左手系。

</details>

## 十、本题怎么用

| 哪一节 | 对应的 TODO |
|---|---|
| 一、记号 | 所有 TODO。写之前先想清楚每个变量是 `T_{?←?}` |
| 二、列向量规则；四、Matrix3x3 按行构造 | `tf_utils::camera_optical_rotation` |
| 五、setRPY | `tf_utils::make_transform` |
| 三、求逆；五、分解；六、四元数互转；七、反解 | `LocatorNode::imu_callback` 里的关节角 |
| 三、复合；四、齐次变换 | `LocatorNode::target_callback` 里把目标抬到世界系 |
| 八、自检 | 单元测试失败时 |

动手之前，`Part_of_Frames.md` 要先读完：那里讲了五段变换各由什么决定，以及固连量从哪来。

第七节的例子里，安装座对应本题的哪个坐标系，IMU 装在哪根连杆上，pan、tilt 分别对应哪个关节，需要你自己对应过去。

---

<div align="right">—— 浙江大学 RoboMaster 算法组</div>
