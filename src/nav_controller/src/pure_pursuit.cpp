// ============================================================================
//  纯追踪控制器实现（本题“控制”的算法核心）。
//
//  本文件实现 computeCommand。函数签名与注释里的契约在
//  include/nav_controller/pure_pursuit.hpp。
// ============================================================================
//
// 算法步骤见 include/nav_controller/pure_pursuit.hpp 头注释，照着实现。
// 完成后运行单元测试：
//   colcon test --packages-select nav_controller
//   colcon test-result --verbose
#include "nav_controller/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav_controller {

Command computeCommand(
    const std::vector<nav_common::Vec2d>& path,
    const nav_common::Pose2d& pose,
    const PurePursuitParams& params)
{
    // TODO：实现路径跟踪控制器：空路径/到达时停车，接近目标时平滑降低目标线速度，
    // 并输出不超过参数上限的有限指令。可以采用纯追踪、Stanley 或其他合理方法。
    // 纯追踪步骤：最近点 -> 前瞻点 -> 车体系 -> 曲率 -> 角速度限幅。

    if (path.empty()) {
        return {0.0, 0.0};
    }

    const double goal_tolerance = std::max(0.0, params.goal_tolerance);
    const double max_linear_velocity = std::max(0.0, params.max_linear_velocity);
    const double max_angular_velocity = std::max(0.0, params.max_angular_velocity);

    // 到达判定：到路径最后一点的距离小于容差就停车。
    const double dx_goal = path.back().x - pose.x;
    const double dy_goal = path.back().y - pose.y;
    const double distance_to_goal = std::hypot(dx_goal, dy_goal);
    if (distance_to_goal < goal_tolerance) {
        return {0.0, 0.0};
    }

    // 1. 最近点：只用于决定从哪里开始向前找前瞻点。
    std::size_t nearest_index = 0;
    double nearest_distance_sq = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.size(); ++i) {
        const double dx = path[i].x - pose.x;
        const double dy = path[i].y - pose.y;
        const double distance_sq = dx * dx + dy * dy;
        if (distance_sq < nearest_distance_sq) {
            nearest_distance_sq = distance_sq;
            nearest_index = i;
        }
    }

    // 2. 前瞻点：从最近点向前扫，取第一个距离 >= lookahead_distance 的点；
    //    如果到末尾都没有，就用最后一个点。
    const double lookahead_distance = std::max(0.0, params.lookahead_distance);
    const double lookahead_distance_sq = lookahead_distance * lookahead_distance;
    std::size_t lookahead_index = path.size() - 1;
    for (std::size_t i = nearest_index; i < path.size(); ++i) {
        const double dx = path[i].x - pose.x;
        const double dy = path[i].y - pose.y;
        if (dx * dx + dy * dy >= lookahead_distance_sq) {
            lookahead_index = i;
            break;
        }
    }

    // 3. 把前瞻点变换到车体系。
    const double dx = path[lookahead_index].x - pose.x;
    const double dy = path[lookahead_index].y - pose.y;
    const double cos_yaw = std::cos(pose.yaw);
    const double sin_yaw = std::sin(pose.yaw);
    const double lx = cos_yaw * dx + sin_yaw * dy;
    const double ly = -sin_yaw * dx + cos_yaw * dy;

    // 4. 曲率。L 是前瞻点的实际距离，不是标称 lookahead_distance。
    const double distance_sq = lx * lx + ly * ly;
    constexpr double kMinDistanceSq = 1.0e-12;
    if (distance_sq <= kMinDistanceSq) {
        return {0.0, 0.0};
    }
    const double curvature = 2.0 * ly / distance_sq;

    // 5. 接近终点时线性减速；slowdown_distance 无效时直接全速。
    double speed_scale = 1.0;
    if (params.slowdown_distance > 0.0) {
        speed_scale = std::min(1.0, distance_to_goal / params.slowdown_distance);
    }
    const double linear_velocity = max_linear_velocity * speed_scale;

    // 6. 角速度 = v * kappa，并限制在配置范围内。
    double angular_velocity = linear_velocity * curvature;
    angular_velocity = std::clamp(
        angular_velocity, -max_angular_velocity, max_angular_velocity);

    if (!std::isfinite(linear_velocity) || !std::isfinite(angular_velocity)) {
        return {0.0, 0.0};
    }

    return {linear_velocity, angular_velocity};
}

} // namespace nav_controller
