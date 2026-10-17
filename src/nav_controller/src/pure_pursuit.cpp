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
    (void)path;
    (void)pose;
    (void)params;
    return {};
}

} // namespace nav_controller
