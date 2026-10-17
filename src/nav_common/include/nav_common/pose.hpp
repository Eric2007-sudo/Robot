// 2D 几何基础类型：点与位姿。
//
// 本项目全程用 2D 平面导航，世界坐标一律是 (x, y) 米、yaw 弧度。这里的类型被
// nav_planner / nav_controller / nav_sim 三边共用，是纯数据结构，不含算法。
#pragma once

#include <cmath>

namespace nav_common {

/// 2D 点（世界坐标，米）
struct Vec2d {
    double x = 0.0;
    double y = 0.0;
};

/// 2D 位姿。yaw 绕 z 轴，单位弧度，正方向遵循右手定则（逆时针为正）。
/// 配合 REP-103（x 朝前、y 朝左），yaw 是车头相对世界 x 轴的夹角。
struct Pose2d {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

/// 把角度归一到 [-pi, pi)。
inline double wrapAngle(double a) {
    constexpr double kPi = 3.14159265358979323846;
    while (a >= kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

/// 两点平面距离（米）
inline double distance(const Vec2d& a, const Vec2d& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

} // namespace nav_common
