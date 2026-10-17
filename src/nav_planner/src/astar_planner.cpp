// ============================================================================
//  A* 全局规划器实现（本题“规划”的算法核心）。
//
//  本文件实现三个函数：gridToWorld / worldToGrid / planPath。
//  函数签名与注释里的契约在 include/nav_planner/astar_planner.hpp。
// ============================================================================
//
// 坐标系/栅格约定见 nav_common/grid_map.hpp 头注释，务必先读懂再动手。
// 完成后运行单元测试：
//   colcon test --packages-select nav_planner
//   colcon test-result --verbose
#include "nav_planner/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace nav_planner {

nav_common::Vec2d gridToWorld(const nav_common::GridMap& map, int col, int row) {
    // TODO 1：返回格子 (col, row) 中心的世界坐标。
    // 契约：x = origin_x + (col + 0.5) * resolution；
    //       y = origin_y + (row + 0.5) * resolution。
    (void)map;
    (void)col;
    (void)row;
    return {};
}

bool worldToGrid(
    const nav_common::GridMap& map, double x, double y, int* col, int* row)
{
    // TODO 2：把世界坐标映射到包含它的格子，写到 *col / *row。
    // 契约：col = floor((x - origin_x) / resolution)，row 同理使用 y；
    //       越界返回 false，此时不要求修改 col/row。
    (void)map;
    (void)x;
    (void)y;
    (void)col;
    (void)row;
    return false;
}

std::vector<nav_common::Vec2d> planPath(
    const nav_common::GridMap& map,
    const nav_common::Vec2d& start,
    const nav_common::Vec2d& goal,
    const PlannerOptions& options)
{
    // TODO 3：在代价图上实现全局规划，返回合法、可执行的路径。
    // 要求：
    //   - 先用 worldToGrid 把起点/终点映射到格子；输出路径使用 gridToWorld 的格子中心。
    //   - 起点或终点不可达时返回空路径。
    //   - 可以使用 A*、Dijkstra 或带安全代价的其他搜索方法。
    //   - 输出路径必须避开障碍；若使用 8 邻域，对角移动要禁止穿过障碍物的墙角。
    //   - 代价图中的 value >= options.occupied_threshold 视为不可通行；默认阈值为 99。
    (void)map;
    (void)start;
    (void)goal;
    (void)options;
    return {};
}

} // namespace nav_planner
