<h1 align="center">Part of C++</h1>

这份文档只讲本题会用到的那几种 C++ 写法。不讲语言规则的全部细节，每一种写法都对应本题代码里实际出现的某一行：它在做什么，写错了会怎样。

Test1 的反馈里有几位同学说，难的不是 ROS 2，而是"知道要调用什么，但写不出调用的格式"。这份文档就是为这个准备的。


## 一、两种智能指针

### shared_ptr：多处共用一个对象

```cpp
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
```

`std::make_shared<T>(参数...)` 在堆上创建一个 `T`，用括号里的参数调用 `T` 的构造函数，返回指向它的 `shared_ptr`。

`shared_ptr` 可以复制，每复制一份，内部的引用计数加一；最后一份销毁时，对象才被释放。所以它适合"好几个地方都要用同一个对象"的场景。

本题的 Buffer 就是这样：节点自己要用它查询，`try_lookup_tf` 也要接收它。`try_lookup_tf` 的第一个参数类型是 `const std::shared_ptr<tf2_ros::Buffer>&`，直接把成员变量传进去：

```cpp
tf_utils::try_lookup_tf(tf_buffer_, ...);
```

### unique_ptr：只有一个主人

```cpp
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
```

`unique_ptr` 不能复制，只能有一个拥有者。它销毁时对象一起销毁。节点里大多数"创建之后只有自己用"的对象都用它，比如广播器和监听器。

### 用指针调用成员函数

```cpp
tf_broadcaster_->sendTransform(msg);
tf_buffer_->canTransform(target, source, stamp);
```

两种智能指针都用 `->` 调用成员函数，和普通指针一样。

### `*tf_buffer_` 和 `this`，一个加 `*` 一个不加

构造函数里这一行来自本题代码：

```cpp
tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);
```

两个参数，一个写 `*tf_buffer_`，一个写 `this`，为什么不一样？看构造函数要什么：

- **第一个参数要 Buffer 的引用**（`tf2::BufferCore&`）。`tf_buffer_` 是 `shared_ptr<Buffer>`，是个"指针"，不能直接当 Buffer 用，所以要 `*tf_buffer_` 把智能指针解开，得到它指向的对象
- **第二个参数要指向节点的指针**。`this` 本身就是指针，直接用；再加 `*` 反而变成对象引用，类型就不对了

第一个参数漏掉 `*`，写成 `(tf_buffer_, this)`，实测报错是：

```
error: no matching function for call to
‘tf2_ros::TransformListener::TransformListener(std::shared_ptr<tf2_ros::Buffer>&, locator::LocatorNode*)’
```

"no matching function for call to" 后面括号里列的是**你实际传进去的类型**。拿它和函数声明里要求的类型比一比，就知道哪个参数不对。这里第一个参数是 `std::shared_ptr<...>&`，而函数要的是 Buffer 本身，于是知道它少了 `*`。

第二个参数反过来：写成 `*this` 是**多余**的。在 Jazzy 上它会直接编不过（Jazzy 的这个构造要求传指针），在 Lyrical 上碰巧能过。所以这里统一写 `this`，两边都对。

## 二、引用

### const 引用：只读，不复制

```cpp
inline tf2::Transform make_transform(
    const tf2::Vector3& translation,
    const double roll,
    ...
)
```

`const tf2::Vector3&` 表示：传进来的是调用者那个对象的引用，函数里不会复制一份；`const` 表示函数里不能改它。

对于 `std::string`、`tf2::Transform` 这类比较大的对象，按 `const T&` 传比按值传省一次复制。`double`、`int` 这种小类型直接按值传就行。

### 输出参数：通过引用把结果带出去

```cpp
inline bool try_lookup_tf(
    ...
    tf2::Transform& tf_out,     // 没有 const
    ...
)
```

没有 `const` 的引用可以在函数里被修改，修改的就是调用者那边的变量。`try_lookup_tf` 用返回值表示"成功没有"，用 `tf_out` 把查到的结果带出去。

调用时先声明一个变量，再把它传进去：

```cpp
tf2::Transform chassis_to_world;
if (!tf_utils::try_lookup_tf(tf_buffer_, ..., chassis_to_world, ...)) {
    return;    // 失败时 chassis_to_world 的内容没有意义，不要用
}
// 走到这里，chassis_to_world 已经被填好了
```

上面的 `chassis_to_world` 如果写成 `&chassis_to_world`，实测的报错是：

```
error: invalid initialization of non-const reference of type ‘tf2::Transform&’
from an rvalue of type ‘tf2::Transform*’
```

意思是：这里要一个 `tf2::Transform&`，你给的是 `tf2::Transform*`。去掉变量前面的 `&`。函数声明里 `tf2::Transform&` 没有 `*`，就说明要传对象本身，不是它的地址。

## 三、lambda

### 订阅回调

```cpp
imu_sub_ = create_subscription<Imu>(
    topic,
    qos,
    [this](const Imu::SharedPtr msg) { imu_callback(msg); },
    options
);
```

`[this](参数) { 函数体 }` 是一个 lambda，也就是一个就地写的匿名函数。方括号是**捕获列表**，决定 lambda 里能用外面的哪些东西。

`imu_callback` 是成员函数，调用它需要知道"是哪个对象的"。`[this]` 把当前对象的指针捕获进来，lambda 里写 `imu_callback(msg)` 就等于 `this->imu_callback(msg)`。

把 `[this]` 写成 `[]`，实测的报错是：

```
error: ‘this’ was not captured for this lambda function
error: cannot call member function
‘void locator::LocatorNode::imu_callback(sensor_msgs::msg::Imu_<std::allocator<void> >::SharedPtr)’
without object
```

第二条里 `Imu_<...>` 是消息类型的完整写法，`Imu` 只是它的短名字。看到"without object"，就是 lambda 没捕获 `this`。

Test1 里用的 `std::bind(&Node::callback, this, std::placeholders::_1)` 和这个 lambda 等价，两种都可以。

### 按引用捕获局部变量

```cpp
const rclcpp::Time stamp = msg->header.stamp;

tf_utils::try_lookup_tf(
    tf_buffer_, frames::WORLD, frames::BASE_LINK, stamp, chassis_to_world,
    [&](const std::string& err) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "lookup failed: %s", err.c_str());
    }
);
```

`[&]` 表示 lambda 里用到的外部变量全部按引用捕获，包括 `this`（所以能调用 `get_logger()`）。

按引用捕获有一个前提：lambda 被调用时，它引用的变量必须还活着。这里 lambda 在 `try_lookup_tf` 内部被立刻调用，函数返回前就用完了，所以没问题。如果把 lambda 存下来以后再调用，局部变量可能已经销毁，那时就不能用 `[&]`。

## 四、std::function：把函数当参数传

```cpp
const std::function<void(const std::string& err_info)>& err_info_handler
```

`std::function<返回类型(参数类型)>` 可以装下任何"能这样调用"的东西：普通函数、lambda、成员函数绑定。`try_lookup_tf` 用它让调用者决定"出错时做什么"：打日志、计数、什么都不做都可以。

本题的单元测试里就有一个，用来把错误信息记下来。它在 `test_tf_utils.cpp` 里：

```cpp
std::function<void(const std::string&)> recorder() {
    return [this](const std::string& err) {
        err_count_++;
        last_err_ = err;          // 把错误信息存下来
    };
}
```

`try_lookup_tf` 内部拿到这个 `std::function` 之后，就像调用普通函数一样调用它，把 `ex.what()` 传进来。

## 五、异常

```cpp
try {
    const auto msg = tf_buffer_->lookupTransform(target, source, time);
    // 查到了
} catch (const tf2::TransformException& ex) {
    // 没查到，ex.what() 是报错文字
}
```

`lookupTransform` 查不到时**抛异常**，不是返回一个特殊值。不 `catch` 的话，异常会一路往上抛，最后让整个容器进程崩掉，五个节点一起退出。

几个要点：

- `catch` 按 `const 类型&` 接，不要按值接
- `tf2::TransformException` 是所有 TF 异常的基类，接住它就接住了 `LookupException`、`ExtrapolationException`、`ConnectivityException` 等所有子类
- `ex.what()` 返回 `const char*`，可以直接转成 `std::string`

`try_lookup_tf` 的契约是"失败时不抛异常，把错误信息交给 handler，返回 false"。所以它要在内部把异常接住。单元测试 `TryLookupTf.ReturnsFalseOnUnknownFrame` 会检查这一点。

## 六、std::vector

本题一次要发两段变换，用的是 `std::vector`。`publish_marker` 里往 `trail_points_` 里积累轨迹也是同一个写法：

```cpp
std::vector<geometry_msgs::msg::Point> trail_points_;
trail_points_.push_back(p);                        // 在末尾追加
trail_points_.erase(trail_points_.begin());        // 删掉最早的一个
```

本题的静态 TF 就是一次发两条，用 `std::vector` 装好，一次交给 `sendTransform`，它们会被装进同一条 `/tf_static` 消息里（`TransformStamped` 由 `tf2::to_transform_stamped` 拼出来）：

```cpp
std::vector<TransformStamped> transforms;
transforms.push_back(tf_utils::to_transform_stamped(tf1, parent1, child1, stamp));
transforms.push_back(tf_utils::to_transform_stamped(tf2, parent2, child2, stamp));
static_broadcaster_->sendTransform(transforms);   // 也能写成 sendTransform({msg1, msg2})
```

`trail_points_` 是成员变量，被回调反复读写。以后让多个线程同时跑这个回调时，这种共享的 `vector` 就是一个要留意的地方，见 `Part_of_ROS2.md` 第二节。

## 七、头文件里的函数要加 inline

```cpp
// tf_utils.hpp
inline tf2::Transform make_transform(...) { ... }
```

`tf_utils.hpp` 是一个只有头文件的库，函数的实现直接写在头文件里。这时**必须加 `inline`**。

原因：头文件会被多个 `.cpp` 包含。不加 `inline`，每个 `.cpp` 都会编译出一份同名函数，链接时报重复定义。在本题上实测（在 `src/locator/src/` 下加一个也包含 `tf_utils.hpp` 的 `.cpp`）：

```
multiple definition of `locator_common::tf_utils::make_transform(tf2::Vector3 const&, double, double, double)';
locator_node.cpp:(.text+0xbc0): first defined here
```

本题现在只有 `locator_node.cpp` 一个源文件包含 `tf_utils.hpp`，所以就算漏了 `inline` 也暂时不报错，`make_transform` 不会真的撞车。但只要多一个源文件包含它，链接立刻失败——单元测试那个源文件就在另一个目标里，暂时也算不到一起。骨架里已经写好了 `inline`，不要删掉。

## 八、几个小知识点

### (void) 是在消除警告

骨架里有这样的行：

```cpp
(void)msg;
(void)tf_buffer;
```

这是告诉编译器"这个参数我故意没用"，否则 `-Wall -Wextra` 会警告 unused parameter。**实现函数时把这些行删掉。** 留着不影响运行，但说明你可能有参数忘了用。

### 命名空间别名

```cpp
namespace frames = locator_common::frames;
namespace tf_utils = locator_common::tf_utils;
```

给长命名空间起个短名字。之后 `frames::WORLD` 就是 `locator_common::frames::WORLD`。

### 时间戳的转换

```cpp
const rclcpp::Time stamp = msg->header.stamp;   // 消息里的时间 → rclcpp::Time
out.header.stamp = stamp;                       // rclcpp::Time → 消息里的时间
```

消息里的 `header.stamp` 类型是 `builtin_interfaces::msg::Time`，只有 `sec` 和 `nanosec` 两个字段。`rclcpp::Time` 是 rclcpp 的时间类，能做加减和比较。两者之间可以直接赋值互转。


## 九、读编译报错

g++ 的报错经常一屏都是模板展开，看起来很吓人。按这个顺序读：

1. **只看第一条 `error:`**。后面的报错常常是第一条引起的连锁反应
2. 看 `error:` 那一行开头的**文件名和行号**。如果是你的文件，直接去那一行；如果是系统头文件（`/usr/include/...`、`/opt/ros/...`），往下找 `required from here`，那里会指回你的代码
3. 读 `error:` 后面的文字。最常见的三种：

| 报错开头 | 意思 |
|---|---|
| `no matching function for call to` | 参数类型对不上。括号里是你实际传的类型，和函数声明比 |
| `invalid initialization of ... reference` | 该传引用的地方传了指针，或者反过来 |
| `is not a member of` | 名字拼错了，或者少了 `#include`、少了命名空间前缀。例如把 `TransformBroadcaster` 打成 `TransformBroadcastr`，报错是 `‘TransformBroadcastr’ is not a member of ‘tf2_ros’; did you mean ‘TransformBroadcaster’?` |

## 十、本题怎么用

| 本文的哪一节 | 出现在哪 |
|---|---|
| 一、智能指针 | 构造函数里的 `tf_buffer_`、`tf_listener_`、两个广播器 |
| 二、输出参数 | `try_lookup_tf` 的 `tf_out`，以及调用它的两个回调 |
| 三、lambda | 构造函数里的两个订阅回调，回调里的 `err_info_handler` |
| 四、std::function | `try_lookup_tf` 的最后一个参数 |
| 五、异常 | `try_lookup_tf` 的实现 |
| 六、vector | 发布两段 TF |
| 七、inline | `tf_utils.hpp` 的五个函数 |
| 八、时间戳的转换 | 两个回调 |

---

<div align="right">—— 浙江大学 RoboMaster 算法组</div>
