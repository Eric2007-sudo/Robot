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
    const double x = map.origin_x + (col + 0.5) * map.resolution;
    const double y = map.origin_y + (row + 0.5) * map.resolution;
    return nav_common::Vec2d{x, y};
}

bool worldToGrid(
    const nav_common::GridMap& map, double x, double y, int* col, int* row)
{
    // TODO 2：把世界坐标映射到包含它的格子，写到 *col / *row。
    // 契约：col = floor((x - origin_x) / resolution)，row 同理使用 y；
    //       越界返回 false，此时不要求修改 col/row。
    if (col == nullptr || row == nullptr) {
        return false;
    }
    if (!std::isfinite(x) || !std::isfinite(y)) {
        return false;
    }

    const int c = static_cast<int>(
        std::floor((x - map.origin_x) / map.resolution));
    const int r = static_cast<int>(
        std::floor((y - map.origin_y) / map.resolution));

    if (!map.inBounds(r, c)) {
        return false;
    }

    *col = c;
    *row = r;
    return true;
}

namespace {

struct Node {
    double f = 0.0;
    double g = 0.0;
    int id = 0;
    int col = 0;
    int row = 0;
};

// std::priority_queue 默认是大顶堆；这里用 a.f > b.f 让小 f 先出。
// f 相同时，g 更大的节点更靠近目标，优先扩展。
struct CompareNode {
    bool operator()(const Node& a, const Node& b) const {
        if (a.f != b.f) {
            return a.f > b.f;
        }
        return a.g < b.g;
    }
};

} // namespace

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

    if (map.width <= 0 || map.height <= 0 || map.resolution <= 0.0 ||
        map.data.size() != static_cast<size_t>(map.width) * map.height) {
        return {};
    }

    int start_col = 0;
    int start_row = 0;
    int goal_col = 0;
    int goal_row = 0;
    if (!worldToGrid(map, start.x, start.y, &start_col, &start_row) ||
        !worldToGrid(map, goal.x, goal.y, &goal_col, &goal_row)) {
        return {};
    }

    if (map.isOccupied(start_row, start_col, options.occupied_threshold) ||
        map.isOccupied(goal_row, goal_col, options.occupied_threshold)) {
        return {};
    }

    const int start_id = map.index(start_row, start_col);
    const int goal_id = map.index(goal_row, goal_col);

    // 起点和终点在同一格：返回该格中心一个点。
    if (start_id == goal_id) {
        return {gridToWorld(map, start_col, start_row)};
    }

    const auto heuristic = [&](int col, int row) {
        const double dx = (col - goal_col) * map.resolution;
        const double dy = (row - goal_row) * map.resolution;
        return std::hypot(dx, dy);
    };

    const size_t cell_count = static_cast<size_t>(map.width) * map.height;
    std::vector<double> g_score(
        cell_count, std::numeric_limits<double>::infinity());
    std::vector<int> came_from(cell_count, -1);

    std::priority_queue<Node, std::vector<Node>, CompareNode> open;

    g_score[start_id] = 0.0;
    open.push(Node{heuristic(start_col, start_row), 0.0, start_id, start_col, start_row});

    // 前 4 个是直方向，后 4 个是对角方向。
    const int dc[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dr[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const int direction_count = options.allow_diagonal ? 8 : 4;
    const double diagonal_cost = map.resolution * std::sqrt(2.0);

    while (!open.empty()) {
        const Node current = open.top();
        open.pop();

        // 同一个格子可能多次入堆；g 比记录值更差的条目已经过期。
        if (current.g > g_score[current.id]) {
            continue;
        }

        if (current.id == goal_id) {
            std::vector<nav_common::Vec2d> path;
            int id = goal_id;
            while (id != -1) {
                const int row = id / map.width;
                const int col = id % map.width;
                path.push_back(gridToWorld(map, col, row));
                if (id == start_id) {
                    break;
                }
                id = came_from[id];
            }
            if (id != start_id) {
                return {};
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int i = 0; i < direction_count; ++i) {
            const bool diagonal = i >= 4;
            const int next_col = current.col + dc[i];
            const int next_row = current.row + dr[i];

            if (!map.inBounds(next_row, next_col)) {
                continue;
            }
            if (map.isOccupied(next_row, next_col, options.occupied_threshold)) {
                continue;
            }
            // 禁止穿墙角：对角移动时，两个相邻直方向格都必须是自由格。
            if (diagonal && options.forbid_corner_cut &&
                (map.isOccupied(current.row, next_col, options.occupied_threshold) ||
                 map.isOccupied(next_row, current.col, options.occupied_threshold))) {
                continue;
            }

            const double step_cost = diagonal ? diagonal_cost : map.resolution;
            const double new_g = current.g + step_cost;
            const int next_id = map.index(next_row, next_col);
            if (new_g >= g_score[next_id]) {
                continue;
            }

            g_score[next_id] = new_g;
            came_from[next_id] = current.id;
            open.push(Node{
                new_g + heuristic(next_col, next_row),
                new_g,
                next_id,
                next_col,
                next_row});
        }
    }

    return {};
}

} // namespace nav_planner
