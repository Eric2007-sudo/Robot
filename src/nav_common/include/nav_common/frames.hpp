// 坐标系名字的唯一来源。仿真与导航节点都从这里取，不要各自写字符串。
//
// 为什么要单独抽出来：nav_sim 发 `map -> odom -> base_link`，导航节点查
// `map -> base_link`，这些字符串必须一字不差。各写各的，拼错一个字母编译器不管，
// 运行时 TF 才报"找不到坐标系"。用常量的话，拼错就是编译错误。
#pragma once

namespace nav_common::frames {

inline constexpr const char* MAP = "map";             // 全局地图系，先验地图所在
inline constexpr const char* ODOM = "odom";           // 里程计系，起点为原点
inline constexpr const char* BASE_LINK = "base_link"; // 车体

} // namespace nav_common::frames
