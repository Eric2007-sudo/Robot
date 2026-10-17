// 占据栅格地图的数据结构。
//
// 这是导航地图最基础的形态：把 2D 空间切成一个个小方格，每个格子存一个"占据程度"。
// 本项目只做静态地图，这份结构由 nav_sim 生成 OccupancyGrid 消息后、在规划侧转换而来，
// 也可以直接用本结构构造（单元测试里就是直接构造的）。
//
// 行列约定（务必记牢，这是本题最容易错的地方）：
//   col（列）沿世界 x 轴增大，row（行）沿世界 y 轴增大。
//   data[row * width + col] 存该格的占据值。
//   origin 是 (row=0, col=0) 这个格子的**左下角**在世界坐标系下的坐标。
//
// 占据值沿用 ROS nav_msgs/OccupancyGrid 的约定：
//     0        自由空间
//     100      真实障碍（致命）
//     (0,100)  有概率占据 / 膨胀圈（本项目的膨胀圈用 50）
//     -1       未知
// 所以规划避障判断用"占据值 >= 阈值"（避开真实障碍和膨胀圈），而"撞没撞"只认 100。
#pragma once

#include <cstdint>
#include <vector>

namespace nav_common {

struct GridMap {
    int width = 0;             // 列数（沿 x 方向的格子数）
    int height = 0;            // 行数（沿 y 方向的格子数）
    double resolution = 0.05;  // 每格边长，单位米
    double origin_x = 0.0;     // (row=0, col=0) 左下角的世界 x 坐标
    double origin_y = 0.0;     // (row=0, col=0) 左下角的世界 y 坐标
    std::vector<int8_t> data;  // 长度 = width * height，行主序

    /// 一维索引
    int index(int row, int col) const { return row * width + col; }

    /// 是否在栅格范围内
    bool inBounds(int row, int col) const {
        return row >= 0 && row < height && col >= 0 && col < width;
    }

    /// 取格子的占据值；越界返回 -1（未知）
    int valueAt(int row, int col) const {
        return inBounds(row, col) ? data[index(row, col)] : -1;
    }

    /// 是否障碍。占据值 >= occupied_threshold 视为障碍（默认 99）。
    /// 代价图中 99 是 inscribed，100 是真实致命障碍。
    bool isOccupied(int row, int col, int occupied_threshold = 99) const {
        return valueAt(row, col) >= occupied_threshold;
    }
};

} // namespace nav_common
