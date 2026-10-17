#include "map_server/map_server_node.hpp"
#include "map_server/costmap_inflation.hpp"
#include "map_server/map_loader.hpp"

#include <chrono>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <nav_common/frames.hpp>

namespace map_server {

namespace {

nav_msgs::msg::OccupancyGrid toOccupancyGrid(
    const nav_common::GridMap& map, const std::string& frame_id)
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = frame_id;
    grid.info.width = static_cast<uint32_t>(map.width);
    grid.info.height = static_cast<uint32_t>(map.height);
    grid.info.resolution = static_cast<float>(map.resolution);
    grid.info.origin.position.x = map.origin_x;
    grid.info.origin.position.y = map.origin_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data = map.data;
    return grid;
}

} // namespace

MapServerNode::MapServerNode(const rclcpp::NodeOptions& options)
: Node("map_server", options) {
    const std::string share = ament_index_cpp::get_package_share_directory("map_server");

    const std::string msgpack_path = share + "/maps/" +
        declare_parameter<std::string>("map.msgpack", "RMUC202605.msgpack");
    const std::string frame_id =
        declare_parameter<std::string>("map.frame_id", nav_common::frames::MAP);

    InflationParams inflation;
    inflation.full_cost_radius_m = declare_parameter<double>(
        "global_map.inflation.full_cost_radius_m", 0.1);
    inflation.cutoff_radius_m = declare_parameter<double>(
        "global_map.inflation.cutoff_radius_m", 0.3);
    inflation.decay_rate_per_m = declare_parameter<double>(
        "global_map.inflation.decay_rate_per_m", 24.0);

    const std::string cost_map_topic = declare_parameter<std::string>(
        "global_map.cost_map_pub_topic", "/map_server/global_cost_map");
    // 代价图按固定频率重发；<=0 表示只发布一次。
    const double pub_freq = declare_parameter<double>("global_map.pub_freq", 1.0);

    const bool debug_enable = declare_parameter<bool>("debug.enable", false);
    const std::string static_map_topic = declare_parameter<std::string>(
        "debug.static_map_pub_topic", "/map_server/debug/static_map");

    const nav_common::GridMap raw = loadMapFile(msgpack_path);
    const nav_common::GridMap cost = inflateCostMap(raw, inflation);
    cost_map_ = toOccupancyGrid(cost, frame_id);
    static_map_ = toOccupancyGrid(raw, frame_id);

    // transient_local（latched）：新订阅者一上来就能拿到；同时按 pub_freq 周期重发，
    // 保留最后一张地图，方便稍后启动的订阅者直接收到。
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    cost_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(cost_map_topic, qos);
    cost_map_pub_->publish(cost_map_);

    if (debug_enable) {
        static_map_pub_ =
            create_publisher<nav_msgs::msg::OccupancyGrid>(static_map_topic, qos);
        static_map_pub_->publish(static_map_);
    }

    if (pub_freq > 0.0) {
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / pub_freq),
            [this]() {
                cost_map_pub_->publish(cost_map_);
                if (static_map_pub_) static_map_pub_->publish(static_map_);
            });
    }

    RCLCPP_INFO(
        get_logger(),
        "global cost map published: %ux%u cells, %.3f m/cell, origin (%.2f, %.2f), pub_freq %.2f Hz",
        cost_map_.info.width, cost_map_.info.height, cost_map_.info.resolution,
        cost_map_.info.origin.position.x, cost_map_.info.origin.position.y, pub_freq);
}

} // namespace map_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(map_server::MapServerNode)
