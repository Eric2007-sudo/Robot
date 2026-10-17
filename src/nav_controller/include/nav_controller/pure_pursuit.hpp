// 纯追踪（Pure Pursuit）局部控制器接口。
//
// 你要实现 src/pure_pursuit.cpp 里的 computeCommand。这份头文件是契约，注释不许改。
// 实现要满足 test/test_pure_pursuit.cpp 的单元测试。
//
// 背景：A* 给出了一条世界系路径，但要真正开动差速底盘，需要每个控制周期算出
// (线速度 v, 角速度 w)。纯追踪是最经典的几何跟踪法：在路径前方选一个"前瞻点"，
// 转一个圆弧去追它。
#pragma once

#include <vector>

#include <nav_common/pose.hpp>

namespace nav_controller {

struct PurePursuitParams {
    double lookahead_distance = 0.5;   // 前瞻距离（米）
    double max_linear_velocity = 1.0;  // 最大线速度（米/秒）
    double slowdown_distance = 1.0;    // 开始接近目标减速的距离（米）
    double max_angular_velocity = 2.0; // 最大角速度（弧度/秒）
    double goal_tolerance = 0.1;       // 到达判定半径（米）
};

struct Command {
    double v = 0.0;  // 线速度（米/秒）
    double w = 0.0;  // 角速度（弧度/秒），逆时针为正
};

/// 纯追踪：给定世界系路径与当前位姿，计算速度指令。
///
/// 行为契约：
///   1. 若 path 为空或已到达路径末端，返回 {0, 0}。
///   2. 未到达时输出有限的差速指令，线速度不超过 max_linear_velocity，并在
///      slowdown_distance 内随目标距离降低。
///   3. 角速度必须限制在 [-max_angular_velocity, max_angular_velocity]。
///   4. 控制器可以使用纯追踪、Stanley 或其他路径跟踪方法；前瞻和曲率公式只是参考实现。
///
/// 纯追踪参考实现注意：kappa 使用 L = 前瞻点实际距离，不是标称 lookahead_distance；
/// ly > 0 表示目标在车体左侧。其他控制算法不必使用这些中间量。
Command computeCommand(
    const std::vector<nav_common::Vec2d>& path,
    const nav_common::Pose2d& pose,
    const PurePursuitParams& params);

} // namespace nav_controller
