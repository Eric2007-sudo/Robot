<h1 align="center">Something</h1>

这份文档讲环境：怎样把本题需要的东西装齐、出了问题怎样确认是不是环境的原因，以及几个工程上的约定。

Test1 结束后收到的反馈里，占用时间最多的往往不是写代码，而是装环境、查"为什么编译过了却跑不起来"。这份文档就先把这些环节讲清楚。

Test1 的 `Something.md` 讲过的内容这里不重复：目录结构、`build/install/log` 是什么、为什么要 source、`.sh` 脚本的作用、一个包由什么组成、git 的用法、命名约定。需要的话回去翻。

## 一、先跑自检

拿到仓库后第一件事：

```bash
bash check_env.sh
```

它检查本题需要的每一样东西：系统版本、ROS 2 发行版、编译工具、ROS 2 包、rosdep、端口、残留进程。缺什么会直接打印修复命令。它只检查，不会改你的系统。

全部通过时最后几行是：

```
==================================================================
  全部通过（21 项）。接下来：
    ./compile.sh
    source install/setup.bash
    ros2 launch locator_launcher sim_only.launch.py
==================================================================
```

**遇到任何环境方面的问题，先跑一遍 `check_env.sh`，提问时把输出一起贴出来。** 这比描述"报了一堆错"能更快定位问题。

## 二、一次装齐

### 两条命令

```bash
sudo apt install ninja-build python3-rosdep
rosdep install --from-paths src --ignore-src -y
```

第一条装编译工具 Ninja 和依赖管理工具 rosdep。第二条让 rosdep 读 `src/` 下所有包的 `package.xml`，把声明了但还没装的依赖一次装齐。

`--ignore-src` 的意思是：`src/` 下自己的包（`locator`、`locator_sim` 这些）不用去系统里找，它们由本仓库编译。

### 为什么 Ninja 要单独装

rosdep 只管 `package.xml` 里声明的依赖。Ninja 不是任何一个包的依赖，而是 `compile.sh` 指定的 CMake 生成器（`-G Ninja`），所以 rosdep 管不到它。

没装 Ninja 时，`compile.sh` 会在编译前直接提示：

```
找不到 ninja。本脚本用 -G Ninja 作为 CMake 生成器，先装上：
    sudo apt install ninja-build
```

如果绕过脚本直接 `colcon build --cmake-args -G Ninja`，看到的是另一段报错：

```
CMake Error: CMake was unable to find a build program corresponding to "Ninja".  CMAKE_MAKE_PROGRAM is not set.
CMake Error: CMAKE_C_COMPILER not set, after EnableLanguage
CMake Error: CMAKE_CXX_COMPILER not set, after EnableLanguage
```

后两行看起来像编译器没装，其实是 Ninja 没装，CMake 连探测编译器这一步都走不下去。

### rosdep 第一次用之前

```bash
sudo rosdep init
rosdep update
```

`init` 只需要做一次，再次执行会报"已经初始化过"，不用管。`update` 要联网下载依赖列表，国内网络下经常失败。

用 fishros 一键安装过 ROS 2 的同学，可能已经装过一个叫 `rosdepc` 的替代工具，用法和 `rosdep` 一样，只是换了国内的源：

```bash
rosdepc update
rosdepc install --from-paths src --ignore-src -y
```

`check_env.sh` 会自动检测你装的是哪一个。

### 本题需要的系统包

如果 rosdep 实在用不了，手动装这些：

```bash
sudo apt install ninja-build \
                 ros-$ROS_DISTRO-tf2-ros \
                 ros-$ROS_DISTRO-tf2-geometry-msgs \
                 ros-$ROS_DISTRO-foxglove-bridge
```

其余的（rclcpp、rclcpp_components、各种消息包）装 ROS 2 桌面版时已经带了。

`foxglove_bridge` 不装的话，编译能过，launch 启动到它时才报：

```
[ERROR] [launch]: Caught exception in launch (see debug for traceback):
package 'foxglove_bridge' not found
```

### 三个名字：apt 包名、ROS 包名、CMake 包名

同一个东西在不同地方叫不同的名字，Test1 里不少人在这里卡过：

| 用在哪 | 本题的例子 |
|---|---|
| `apt install` | `ros-jazzy-tf2-ros`、`ros-jazzy-foxglove-bridge` |
| `package.xml`、`ros2 pkg prefix`、launch 里的 `package=` | `tf2_ros`、`foxglove_bridge` |
| CMakeLists 里的 `find_package` | `tf2_ros` |

ROS 包的 apt 名规律是：`ros-<发行版>-<包名>`，包名里的下划线换成短横线。

## 三、发行版

本题以 **Ubuntu 24.04 + ROS 2 Jazzy** 为主线编写，在 **Ubuntu 26.04 + ROS 2 Lyrical** 上验证过。两个发行版上代码都不用改。

已知的差异有两处：

**一、`component_container_mt` 的弃用提示。** Lyrical 上启动它会打印一条

```
This executable is deprecated and will be removed in M-turtle.
Use 'component_container --executor-type multi-threaded' instead.
```

这是提示，不是错误，不影响运行。Jazzy 上没有这条，而且 Jazzy 的 `component_container` 还不支持 `--executor-type`，所以本题统一用 `component_container_mt`。

**二、`TransformListener` 的第二个参数。** 创建 TF 监听写的是
`std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this)`，第二个参数传的是**指针**
`this`，不是 `*this`。两个发行版在这点上要求不同：Jazzy 的这个构造只接受指针，写成 `*this`
直接编不过；Lyrical 上 `*this` 碰巧也能过。所以统一写 `this`，两边都对。骨架里已经这样写了。

## 四、source 与编译

### 每个新终端都要 source

```bash
source install/setup.bash
```

没有这一句，`ros2 launch locator_launcher ...` 会说找不到包。原因见 Test1 的 `Something.md` 第一节。

`source /opt/ros/$ROS_DISTRO/setup.bash` 一般已经写在 `~/.bashrc` 里，不用再敲。确认方法：新终端里 `echo $ROS_DISTRO` 有输出就说明已经生效。没有输出时，`check_env.sh` 第一项就会报出来。

### 改了什么需要重新编译

`compile.sh` 用了 `--symlink-install`，`install/` 里的 yaml 和 launch 是指向 `src/` 的软链接。

| 改了什么 | 要不要重新编译 |
|---|---|
| 已有的 `.yaml`、`.launch.py`、`.py` 脚本 | 不用，重启节点就生效 |
| `.cpp`、`.hpp` | 要 |
| **新建**了一个 launch 文件或脚本 | **要**，编译一次才会出现在 `install/` 里 |
| `CMakeLists.txt`、`package.xml` | 要 |

第三行最容易踩。软链接是编译时建的，编译之后新建的文件在 `install/` 里还不存在。你第一次写 `locator_launcher.launch.py` 时，写完要先 `./compile.sh` 一次，`ros2 launch` 才能找到它。之后再改它就不用重新编译了。

### 只编一个包

```bash
colcon build --symlink-install --cmake-args -G Ninja --packages-select locator
```

只改了 `locator_node.cpp` 时用这条，比全量编译快。注意 `tf_utils.hpp` 在 `locator_common` 包里，改了它要连 `locator_common` 一起编：

```bash
colcon build --symlink-install --cmake-args -G Ninja --packages-select locator_common locator
```

### 跑单元测试

```bash
colcon test --packages-select locator_common
colcon test-result --verbose
```

第二条列出每个失败的测试和原因。测试代码在 `src/locator_common/test/test_tf_utils.cpp`，每个测试前面都写了它检查的是什么。

## 五、按报错查

把报错里的关键文字复制下来，在本页搜。

### 编译阶段

| 报错 | 原因 | 处理 |
|---|---|---|
| `CMAKE_C_COMPILER not set, after EnableLanguage` | 没装 Ninja | `sudo apt install ninja-build` |
| `Could not find a package configuration file provided by "tf2_ros"` | 缺 ROS 包 | `rosdep install --from-paths src --ignore-src -y` |
| `no matching function for call to` | 参数类型对不上 | 见 `Part_of_Cpp.md` 第九节 |

### 启动阶段

| 报错 | 原因 | 处理 |
|---|---|---|
| `Package 'locator_launcher' not found` | 当前终端没 source | `source install/setup.bash` |
| `file 'locator_launcher.launch.py' was not found in the share directory` | 新建 launch 后没重新编译 | `./compile.sh` |
| `package 'foxglove_bridge' not found` | 没装 bridge | `sudo apt install ros-$ROS_DISTRO-foxglove-bridge` |
| `Statically typed parameter '...' must be initialized` | yaml 没传进去 | 见 `Part_of_ROS2.md` 第七节 |
| `Failed to find class with the requested plugin name` | launch 里 plugin 写错 | 见 `Part_of_ROS2.md` 第七节 |

### 运行阶段

| 日志 | 原因 | 处理 |
|---|---|---|
| `offering incompatible QoS. No messages will be sent to it` | 订阅和发布的 QoS 不兼容 | 见 `Part_of_ROS2.md` 第一节 |
| `tf chain world -> camera_optical not available` 持续出现 | TF 链路缺了一段 | 见 `Part_of_TF.md` 第九节 |
| `chassis pose not available at imu stamp` 持续出现 | 查询时间不对 | 见 `Part_of_TF.md` 第四节 |
| `unexpected residual roll` 持续出现 | 反解公式错了，或者时间没对齐 | 见 `Part_of_Robotics_Math.md` 第七节 |
| 启动瞬间出现一两次上面三条之一，之后不再出现 | 正常。启动时数据还没到齐 | 不用处理 |

### 端口被占用

```
Address already in use
```

上一次的 foxglove_bridge 没退干净。

```bash
pkill -f foxglove_bridge
```

## 六、残留进程

`Ctrl+C` 停掉 launch 时，偶尔会有子进程没退出。残留的仿真节点会继续发数据，和这次启动的节点混在一起，结果会莫名其妙地不对：误差忽大忽小，或者出现两份 TF。

每次重新启动前确认一下：

```bash
pgrep -af component_container
```

有输出就清掉：

```bash
pkill -f component_container
pkill -f foxglove_bridge
```

`check_env.sh` 的最后一项也会检查这个。

同一个局域网里，别人跑的 ROS 2 节点也可能被你的电脑收到。宿舍或实验室网络里多人同时调试时，每人设一个不同的域 ID：

```bash
export ROS_DOMAIN_ID=42      # 0~101 之间任选一个，每个终端都要设，或者写进 ~/.bashrc
```

## 七、三个校验脚本

本题的验收不看"看着像不像"，看三个脚本给出的数字。它们都在 `locator_launcher/scripts/` 下，安装之后用 `ros2 run locator_launcher` 调用。

三个脚本从不同角度切入同一条链路。整条链路有五段变换，任何一段错了最终结果都会偏，但从结果看不出错在哪一段；三个脚本把"哪里错了"缩小到"哪一段错了"。

### 运行方式

三个脚本都要在**系统正在运行时**跑。先开一个终端把系统跑起来：

```bash
source install/setup.bash
ros2 launch locator_launcher locator_launcher.launch.py
```

再开一个终端跑脚本：

```bash
source install/setup.bash
ros2 run locator_launcher verify_gimbal.py
```

每个脚本默认采集 25 秒。想改时长用参数传，注意 `duration` 是浮点数，写 `-p duration:=20` 会报类型错误，要写 `20.0`：

```bash
ros2 run locator_launcher verify_accuracy.py --ros-args -p duration:=20.0
```

### verify_gimbal.py：先看两个电机角

它只看云台的两段 TF：`base_link → gimbal_yaw` 和 `gimbal_yaw → gimbal_pitch`。

仿真的虚拟 IMU 在发 IMU 消息的同时，用同一个时间戳另发一路关节角真值。脚本按时间戳把你的 TF 和真值配对，**只比较旋转**（平移是固连量，直接用 `tf2_echo` 看更清楚）。输出长这样：

```
  yaw  ：配对  7500 个，平均   0.27 mdeg，最大   1.13 mdeg
  pitch：配对  7500 个，平均   0.48 mdeg，最大   1.53 mdeg
  通过：两个关节角都在容差 57.296 mdeg 内
```

`mdeg` 是千分之一度。正确写法误差在 1~2 mdeg，容差给到 57 mdeg，留了很大余量。

它最大的价值是把问题**分成两半**：

- 它过了、`verify_accuracy` 没过：电机角是对的，问题出在静态 TF 或 target 回调
- 它没过：问题在 imu 回调，去查反解公式、查询时间、帧序

配不上样本时它也会给线索：

| 它说 | 说明 |
|---|---|
| 某个关节角"没有配对到样本" | 那一段 TF 没发、父子写反、或 frame 名字拼错 |
| "看到了反向的 TF" | 父子确实写反了 |
| 两个都配上了但误差是几万 mdeg | 反解公式错（复合顺序、转置） |
| 误差随时间变大，量级在几十到上百 mdeg | 查询 TF 用的时间不对 |

### verify_accuracy.py：整条链的终点

它比较你算出的**世界系目标位置**和真值，按时间戳逐点配对。这是最终要看的那一项：

```
  配对样本数     : 1200
  平均误差       : 0.016399 mm
  最大误差       : 0.144717 mm
  通过：最大误差 0.144717 mm 在容差 1.000 mm 内
```

容差 1 mm。仿真没有噪声，正确写法在 0.02~0.3 mm，比容差小一个量级。

两个数字要看：**配对样本数**和**未配对**。未配对不为零，说明有一批帧没算出来，通常是时间戳对不上；配对数是 0，说明链路根本没通，先别管误差，去查链路。

### verify_cassini.py：不看真值的独立检查

前两个都依赖仿真的真值。第三个不依赖：它只订阅你的输出，检验这些点是否满足卡西尼卵形线的定义——到两个焦点的距离之积恒为常数。

```
  样本数             : 1500
  相对误差最大       : 0.001789 %
  极角覆盖           : -179.8° .. +179.9°，最大间隙 0.34°
  通过：轨迹满足卡西尼卵形线定义
```

它检查两件事：乘积是否恒定（形状对不对），极角是否覆盖一整圈（有没有缺一段）。用处是：即使真值那条链路有问题，它也能独立告诉你输出是不是一条合格的卵形线。

### 三个一起看

按 `gimbal → accuracy → cassini` 的顺序跑，对照下表：

| 现象 | 结论 |
|---|---|
| 三个都过 | 完成 |
| gimbal 过、accuracy 不过 | 电机角对，查静态 TF 和 target 回调 |
| gimbal 不过 | 查 `imu_callback` |
| accuracy 和 cassini 都配不上样本 | 链路断了或时间戳错，看 `Part_of_TF.md` 第九节 |
| 配上了但误差几十毫米 | 多半是时间问题，不是公式问题 |

这套脚本也在示范一种做法：**每完成一步就单独验证一次**，而不是全部写完再来。你每写完一个函数，就有一个脚本能告诉你它对不对。

### 话题名是约定好的

三个脚本订阅的话题名在代码里写死（`locator/target_in_world`、`locator_sim/detector/target_truth` 等）。这是题目约定的名字，不要改。如果你改了 `locator_params.yaml` 里的话题名，跑脚本时要用 `-p located_topic:=...` 之类把新名字传进去，否则它配不上，会报"没有配对到任何样本"。

## 八、仓库为什么这样分

打开 `src/`，会看到四个包。为什么不把代码全塞进一个包？

一个包是一份能单独编译、单独安装、单独被别人依赖的东西。本题这四份代码角色不同，分清了角色，出问题时才知道该看哪里、该重新编译哪个。

| 包 | 角色 | 谁动它 |
|---|---|---|
| `locator_common` | 两边共用的名字和工具 | 你补 `tf_utils.hpp` |
| `locator_sim` | 四个虚拟硬件节点，模拟真车上会接的东西 | 只读 |
| `locator` | 被测主体，你要写的节点 | 你写 |
| `locator_launcher` | 怎么把整套系统跑起来，外加校验工具 | 你写 launch |

### locator_common：两边共同的约定

坐标系的名字（`frames.hpp`）和几个变换工具（`tf_utils.hpp`）放在这个包里。

为什么要单独抽出来？因为**仿真和 locator 必须用同一套名字**。仿真发 `world → base_link`，locator 查 `world → base_link`，这两个字符串得一模一样。各写各的，拼错一个字母编译器不管，运行时才找不到坐标系。把名字放进一个头文件、两边都 `#include`，就只剩一个地方可能写错。

它是**纯头文件**的包，没有 `.cpp`，因为它只提供定义和很短的函数，没有需要编译的东西。谁 `#include` 谁就能用。

可以把它看成一个约定：仿真和 locator 必须用同一套坐标系名字，名字集中写在这里，两边都从这里取。

### locator_sim：车上的硬件

四个节点分别模拟串口、IMU、detector 和可视化。真车上就是这四样东西：下位机通过串口发底盘位姿，IMU 发云台姿态，相机那边跑检测算法，还有一个给人看的显示界面。仿真把它们都做成了节点。

为什么单独一个包，还标成只读？一是它在扮演"硬件"，你不该去改硬件；二是它同时是校验的裁判，里面的真值不能被你的代码影响（原因见第九节"裁判不依赖被测代码"）。

### locator：你要写的

只有一个节点和它的参数文件。题目在 `locator_common` 里留了五个函数、在 `locator` 里留了四个 TODO。它依赖 `locator_common` 取名字和工具，通过话题和 TF 跟 `locator_sim` 打交道。

### locator_launcher：怎么把系统跑起来

这里放 launch 文件和校验脚本。为什么不把 launch 放进 `locator` 包？

因为 launch 是**运行方式**，不是**节点本身**。同一个 `locator` 节点可以有好几种跑法：只跑仿真调试、跑完整系统、换一组参数做实验。这些都属于"怎么用"，不属于"节点是什么"。分开之后，`locator` 包只回答一个问题：这个节点做什么。

### 为什么是一个大 launch

本题的 launch 把四个仿真节点、locator、foxglove_bridge 一起拉起来，塞进**一个容器**。这不是图省事，是硬性要求：

- 四个仿真节点靠一个进程内共享的起点对齐仿真时间（见 `Part_of_ROS2.md` 第三节）。拆成多个进程不会报错，但各节点描述的世界会不一致
- 一个入口，一条命令起全套。调试时反复启动，多敲几条命令容易漏
- `override_params` 要同时作用到仿真和 locator 两边，写在一个 launch 里最直接

对照 Test1：那题只有一个节点，用 `Node` 单独起来就够了。本题节点多、还要共享进程，才需要容器和这个稍大的 launch。给你参照的 `sim_only.launch.py` 就是它的简化版，去掉 locator 和那些参数。

## 九、几个工程约定

### 名字只写一次

所有坐标系的名字定义在 `locator_common/include/locator_common/frames.hpp`。仿真和 locator 都从这里取，代码里用 `frames::WORLD`，不写字符串 `"world"`。

字符串拼错一个字母，编译器发现不了，运行时 tf2 只会说找不到这个坐标系。用常量的话，拼错就是编译错误。

### 两边的安装量必须一致

`locator_params.yaml` 里的 `mount.*` 和 `sim_params.yaml` 里的 `scene.*` 描述的是同一台车的同一组安装量，必须一致。

真实的车上，这些值来自机械图纸或者标定，locator 自己没法测出来。这是一份必须人工维护的契约。本题批改时用 `override_params` 换一组安装量，两边会同时改。

### 裁判不依赖被测代码

仿真节点发布 TF 和真值时，用的是自己的 `msg_conversions.hpp`，不用 `tf_utils.hpp`。`camera_optical_rotation` 在 `scene_model.hpp` 里也单独实现了一份。

这是故意的。仿真是校验的裁判。如果它也调用 `tf_utils`，你写错一个转换函数，真值和你的结果会被同一个错误污染，逐点比对反而显示"通过"。

## 十、关于 AI

Test1 里很多同学用了 AI。这很正常，查资料、读报错、补全代码，AI 都很有用。

但本题有几件事交给 AI 会让任务失去意义：

- **推导反解公式。** `R_baseᵀ · R_imu` 这一行是本题的核心。自己在纸上推一遍，和让 AI 直接写出来，收获完全不同。报告里有一道手算题，要求拍照上传推导过程
- **决定查询用哪个时间戳。** 这需要理解每路数据的时间戳是什么含义，AI 不知道本题仿真的延迟是怎样设计的
- **判断代码对不对。** 能编译、能运行、Foxglove 里看着像，都不等于对。本题有几种错误写法能通过所有这些检查，只有校验脚本能抓到

适合交给 AI 的：

- 某个 API 的签名和用法（但要对照本题的 ROS 2 版本确认）
- 解释一段看不懂的报错
- 补 `#include`、修语法错误

报告里有几道题需要你实际改一次代码、跑一次校验，把结果贴出来。这类题 AI 可以给出推测，但给不出你机器上实际跑出来的数字。

---

<div align="right">—— 浙江大学 RoboMaster 算法组</div>
