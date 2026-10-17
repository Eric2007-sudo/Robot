// A* 全局规划器的接口。
//
// 你要实现 src/astar_planner.cpp 里的三个函数。这份头文件是契约，注释不许改。
// 实现要满足 test/test_astar_planner.cpp 的单元测试。
//
// 背景：给定一张占据栅格地图、起点和目标的世界坐标，找一条不碰障碍的可执行路径。
// 这是导航"怎么走"的第一步：先在有障碍的世界上搜出一条能走的路（拓扑 + 几何），
// 后续的局部控制器再负责把它真正走稳。
#pragma once

#include <vector>

#include <nav_common/grid_map.hpp>
#include <nav_common/pose.hpp>

namespace nav_planner {

struct PlannerOptions {
    bool allow_diagonal = true;     // 是否允许对角移动（8 邻域）
    bool forbid_corner_cut = true;  // 是否禁止"穿墙角"（两个对角障碍夹一个角时不允许斜穿）
    int occupied_threshold = 99;    // 代价值 >= 该阈值视为障碍（inscribed + lethal）
};

/// 栅格 -> 世界：返回格子 (col, row) 中心点的世界坐标。
///
/// 约定（见 nav_common/grid_map.hpp 头注释）：
///   col 沿世界 x 轴，row 沿世界 y 轴；origin 是 (row=0, col=0) 的左下角。
/// 格子的中心 = origin + (col + 0.5) * resolution 沿 x，(row + 0.5) * resolution 沿 y。
nav_common::Vec2d gridToWorld(const nav_common::GridMap& map, int col, int row);

/// 世界 -> 栅格：把世界坐标 (x, y) 映射到包含它的格子 (col, row)。
/// 落在格子边界上的点属于"右下"方向的那一格（floor 语义）。
/// 越界（不在地图覆盖范围内）时返回 false，此时 *col / *row 的值未定义。
bool worldToGrid(const nav_common::GridMap& map, double x, double y, int* col, int* row);

/// 在世界坐标（米）上做全局搜索，返回一条合法、可执行的路径。
///
/// - start / goal 是世界坐标，内部先 snap 到各自所在格子的中心。
/// - 返回值是世界坐标的路径点序列，含起点与终点。
/// - 起点或终点落在障碍内、或者二者之间不可达时，返回空 vector。
///
/// 实现要求：
///   1. 可以使用 A*、Dijkstra、带安全代价的搜索或其他等价的全局规划方法。
///   2. 路径必须避开 occupied_threshold 以上的格子并保持段连通；若使用 8 邻域，
///      对角移动额外要求两个相邻的直方向格子都不是障碍（禁止穿墙角）。
///   3. 障碍判定用 options.occupied_threshold，比较的是"大于等于"，不是"等于 100"。
std::vector<nav_common::Vec2d> planPath(
    const nav_common::GridMap& map,
    const nav_common::Vec2d& start,
    const nav_common::Vec2d& goal,
    const PlannerOptions& options = PlannerOptions{});

} // namespace nav_planner
