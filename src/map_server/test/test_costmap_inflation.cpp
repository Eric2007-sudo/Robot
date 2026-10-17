#include <gtest/gtest.h>

#include <nav_common/grid_map.hpp>
#include <map_server/costmap_inflation.hpp>

namespace {

nav_common::GridMap makeMap() {
    nav_common::GridMap map;
    map.width = 21;
    map.height = 21;
    map.resolution = 0.1;
    map.data.assign(static_cast<size_t>(map.width) * map.height, 0);
    map.data[map.index(10, 10)] = 100;
    return map;
}

map_server::InflationParams params() {
    map_server::InflationParams p;
    p.full_cost_radius_m = 0.2;
    p.cutoff_radius_m = 0.5;
    p.decay_rate_per_m = 4.0;
    return p;
}

} // namespace

TEST(CostmapInflation, KeepsLethalObstacle) {
    const auto cost = map_server::inflateCostMap(makeMap(), params());
    EXPECT_EQ(cost.valueAt(10, 10), 100);
}

TEST(CostmapInflation, UsesEuclideanDistance) {
    const auto cost = map_server::inflateCostMap(makeMap(), params());

    // (10,12) 距离 0.2m，在满代价半径内；(12,12) 距离约 0.283m，在渐变区。
    EXPECT_EQ(cost.valueAt(10, 12), 99);
    EXPECT_LT(cost.valueAt(12, 12), 99);
    EXPECT_GT(cost.valueAt(12, 12), 0);
}

TEST(CostmapInflation, IsSymmetric) {
    const auto cost = map_server::inflateCostMap(makeMap(), params());
    EXPECT_EQ(cost.valueAt(10, 14), cost.valueAt(10, 6));
    EXPECT_EQ(cost.valueAt(14, 10), cost.valueAt(6, 10));
    EXPECT_EQ(cost.valueAt(13, 13), cost.valueAt(7, 7));
}

TEST(CostmapInflation, ReachesZeroAfterCutoff) {
    const auto cost = map_server::inflateCostMap(makeMap(), params());
    EXPECT_EQ(cost.valueAt(10, 17), 0);  // 距离 0.7m，超过截断半径
}
