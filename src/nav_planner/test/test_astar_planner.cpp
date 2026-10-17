// A* 规划器单元测试。
//
// 这里对 gridToWorld / worldToGrid 做数值级校验（不依赖 planPath），对 planPath 用
// 一份**测试内独立实现**的换算来验证路径落格是否正确。这样即使你的换算和 planPath
// 犯了同一个错，也能被这里抓出来。
//
// 运行：colcon test --packages-select nav_planner && colcon test-result --verbose
#include <cmath>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <nav_common/grid_map.hpp>
#include <nav_planner/astar_planner.hpp>

using nav_common::GridMap;
using nav_common::Vec2d;
using nav_planner::planPath;
using nav_planner::gridToWorld;
using nav_planner::worldToGrid;

namespace {

GridMap makeMap(int width, int height, double res = 1.0,
                double ox = 0.0, double oy = 0.0) {
    GridMap m;
    m.width = width;
    m.height = height;
    m.resolution = res;
    m.origin_x = ox;
    m.origin_y = oy;
    m.data.assign(static_cast<size_t>(width) * height, 0);
    return m;
}

// 测试内独立的世界 -> 格换算（按 grid_map.hpp 约定，与学生代码无关）
std::pair<int, int> toCell(const GridMap& m, double x, double y) {
    int c = static_cast<int>(std::floor((x - m.origin_x) / m.resolution));
    int r = static_cast<int>(std::floor((y - m.origin_y) / m.resolution));
    return {c, r};
}

// 验证一条路径是否合法：点和连续线段都不进入障碍格。
bool isPathValid(const GridMap& m, const std::vector<Vec2d>& path, int thresh = 50) {
    if (path.empty()) return false;
    const auto occupied = [&](int c, int r) {
        return !m.inBounds(r, c) || m.valueAt(r, c) >= thresh;
    };
    for (const auto& point : path) {
        const auto [c, r] = toCell(m, point.x, point.y);
        if (occupied(c, r)) return false;
    }
    for (size_t i = 1; i < path.size(); ++i) {
        const double length = std::hypot(
            path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
        const int samples = std::max(1, static_cast<int>(
            std::ceil(length / (m.resolution * 0.5))));
        auto [previous_col, previous_row] = toCell(m, path[i - 1].x, path[i - 1].y);
        for (int sample = 1; sample <= samples; ++sample) {
            const double t = static_cast<double>(sample) / samples;
            const double x = path[i - 1].x + t * (path[i].x - path[i - 1].x);
            const double y = path[i - 1].y + t * (path[i].y - path[i - 1].y);
            const auto [col, row] = toCell(m, x, y);
            if (occupied(col, row)) return false;
            if (std::abs(col - previous_col) == 1 && std::abs(row - previous_row) == 1 &&
                (occupied(previous_col, row) || occupied(col, previous_row))) {
                return false;
            }
            previous_col = col;
            previous_row = row;
        }
    }
    return true;
}

} // namespace

TEST(GridWorld, GridToWorldBasics) {
    GridMap m = makeMap(10, 10, 0.1, -0.5, -0.5);  // 覆盖 [-0.5, 0.5]^2
    auto v = gridToWorld(m, 0, 0);
    EXPECT_NEAR(v.x, -0.45, 1e-9);
    EXPECT_NEAR(v.y, -0.45, 1e-9);

    auto v2 = gridToWorld(m, 9, 9);
    EXPECT_NEAR(v2.x, 0.45, 1e-9);
    EXPECT_NEAR(v2.y, 0.45, 1e-9);
}

TEST(GridWorld, WorldToGridBasics) {
    GridMap m = makeMap(10, 10, 0.1, -0.5, -0.5);
    int c = -1, r = -1;
    ASSERT_TRUE(worldToGrid(m, -0.45, -0.45, &c, &r));
    EXPECT_EQ(c, 0);
    EXPECT_EQ(r, 0);

    ASSERT_TRUE(worldToGrid(m, 0.0, 0.0, &c, &r));  // 场地中心
    EXPECT_EQ(c, 5);
    EXPECT_EQ(r, 5);
}

TEST(GridWorld, WorldToGridOutOfBounds) {
    GridMap m = makeMap(10, 10, 0.1, -0.5, -0.5);
    int c = 0, r = 0;
    EXPECT_FALSE(worldToGrid(m, 0.6, 0.0, &c, &r));   // x 越界
    EXPECT_FALSE(worldToGrid(m, 0.0, -0.6, &c, &r));  // y 越界
}

TEST(PlanPath, StraightLineIsValid) {
    GridMap m = makeMap(11, 11, 0.1, 0.0, 0.0);
    auto path = planPath(m, {0.05, 0.05}, {0.95, 0.05});  // 格 (0,0) -> (9,0)
    ASSERT_FALSE(path.empty());
    EXPECT_NEAR(path.front().x, 0.05, 1e-9);
    EXPECT_NEAR(path.back().x, 0.95, 1e-9);
    EXPECT_TRUE(isPathValid(m, path));
}

TEST(PlanPath, DiagonalIsValid) {
    GridMap m = makeMap(11, 11, 0.1, 0.0, 0.0);
    auto path = planPath(m, {0.05, 0.05}, {1.05, 1.05});  // 格 (0,0) -> (10,10)
    ASSERT_FALSE(path.empty());
    EXPECT_TRUE(isPathValid(m, path));
}

TEST(PlanPath, AvoidsObstacle) {
    // 4x3 地图，res=1，障碍格 (2,1)。起点 (0,1) 目标 (3,1)。
    // 禁止穿墙角时，路径必须绕开障碍并保持相邻格连通。
    GridMap m = makeMap(4, 3, 1.0, 0.0, 0.0);
    m.data[m.index(1, 2)] = 100;  // 障碍 (col=2, row=1)

    auto path = planPath(m, {0.5, 1.5}, {3.5, 1.5});
    ASSERT_FALSE(path.empty());
    EXPECT_TRUE(isPathValid(m, path));      // 不落障碍、对角不穿墙角
}

TEST(PlanPath, CannotCutBlockedCorner) {
    GridMap m = makeMap(2, 2, 1.0, 0.0, 0.0);
    m.data[m.index(0, 1)] = 100;
    m.data[m.index(1, 0)] = 100;
    const auto path = planPath(m, {0.5, 0.5}, {1.5, 1.5});
    EXPECT_TRUE(path.empty());
}

TEST(PlanPath, StartInsideObstacle) {
    GridMap m = makeMap(5, 5, 1.0, 0.0, 0.0);
    m.data[m.index(2, 2)] = 100;
    auto path = planPath(m, {2.5, 2.5}, {0.5, 0.5});
    EXPECT_TRUE(path.empty());
}

TEST(PlanPath, GoalInsideObstacle) {
    GridMap m = makeMap(5, 5, 1.0, 0.0, 0.0);
    m.data[m.index(2, 2)] = 100;
    auto path = planPath(m, {0.5, 0.5}, {2.5, 2.5});
    EXPECT_TRUE(path.empty());
}

TEST(PlanPath, Unreachable) {
    // 一堵墙把左右完全隔开。
    GridMap m = makeMap(11, 5, 1.0, 0.0, 0.0);
    for (int r = 0; r < 5; ++r) m.data[m.index(r, 5)] = 100;
    auto path = planPath(m, {0.5, 2.5}, {10.5, 2.5});
    EXPECT_TRUE(path.empty());
}

TEST(PlanPath, NonTrivialOriginResolution) {
    // 换算用非平凡 origin / resolution，验证路径落格仍正确。
    GridMap m = makeMap(20, 20, 0.1, -1.0, -1.0);  // 覆盖 [-1, 1]^2
    m.data[m.index(10, 10)] = 100;                 // 中心格是障碍
    auto path = planPath(m, {-0.95, -0.95}, {0.95, 0.95});
    ASSERT_FALSE(path.empty());
    EXPECT_TRUE(isPathValid(m, path));
    EXPECT_NEAR(path.front().x, -0.95, 1e-9);
    EXPECT_NEAR(path.back().x, 0.95, 1e-9);
}
