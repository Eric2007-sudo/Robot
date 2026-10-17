// 地图服务器节点。加载先验地图，生成代价图并发布。只读，不要改。
//
//   读取比赛地形标注文件（原始障碍） -> 膨胀成连续代价图
//   -> 发 /map_server/global_cost_map
//
// 参数放在 global_map.* 下，代价图按 pub_freq 周期重发；还可以在 debug.* 下
// 额外发布一份未膨胀的静态图。
#pragma once

#include <memory>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

namespace map_server {

class MapServerNode : public rclcpp::Node {
public:
    explicit MapServerNode(const rclcpp::NodeOptions& options);

private:
    nav_msgs::msg::OccupancyGrid cost_map_;
    nav_msgs::msg::OccupancyGrid static_map_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace map_server
