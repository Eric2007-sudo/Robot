// 地图加载：读比赛地形标注文件，得到原始占据图（0 自由 / 100 障碍）。
//
// 只保留 Test4 需要的静态障碍语义；坡、台阶和飞坡视为可通行区域。
#pragma once

#include <string>

#include <nav_common/grid_map.hpp>

namespace map_server {

/// 读 terrain msgpack，返回原始占据图（无膨胀）：
///   - terrain == OBSTACLE 映射为 100，其余 terrain 标签映射为 0；
///   - msgpack 的行序直接对应地图坐标，row=0 对应世界 y 最小的一侧。
nav_common::GridMap loadMapFile(
    const std::string& msgpack_path);

} // namespace map_server
