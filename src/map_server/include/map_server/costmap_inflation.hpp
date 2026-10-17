// 代价图膨胀：把原始障碍（100）向外膨胀成连续代价梯度。
//
//   - 障碍本体：100（lethal）
//   - 满代价半径内：99（inscribed，规划器避开）
//   - 满代价半径到截断半径：1..98（连续衰减）
//   - 截断半径之外：0
// 只读，不要改。
#pragma once

#include <nav_common/grid_map.hpp>

namespace map_server {

struct InflationParams {
    double full_cost_radius_m = 0.1;  // 满代价半径，代价为 99
    double cutoff_radius_m = 0.3;     // 截断半径，超过后代价为 0
    double decay_rate_per_m = 24.0;   // 距离衰减率
};

/// 输入原始占据图（0 自由 / 100 障碍），输出代价图（0 / 1..99 / 100）。
nav_common::GridMap inflateCostMap(
    const nav_common::GridMap& raw, const InflationParams& params);

} // namespace map_server
