// 导航执行节点。把 A* 规划（nav_planner）和纯追踪控制（nav_controller）粘起来，
// 串成一条完整的导航闭环：
//
//   /map_server/global_cost_map（OccupancyGrid）──┐
//   /nav_goal（PointStamped）─────────────────────┤→ 规划 → /nav_executor/global_path → 纯追踪 → cmd_vel
//   TF map->base_link ────────────────────────────┘                                         └→ robot_sim
//
// 订阅 map_server 发出的 global_cost_map，输出 global_path（计划路径）。
//
// 节点负责把代价图、目标、TF、A* 和纯追踪接成一条完整导航链路。
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <nav_common/grid_map.hpp>
#include <nav_common/pose.hpp>

namespace nav_executor {

class NavExecutorNode : public rclcpp::Node {
public:
    explicit NavExecutorNode(const rclcpp::NodeOptions& options);

private:
    // ---- 你要实现的 ----
    /// OccupancyGrid 消息 -> GridMap 结构体（width/height/resolution/origin/data 原样拷贝）
    static nav_common::GridMap toGridMap(const nav_msgs::msg::OccupancyGrid& msg);

    /// 查 TF 拿 map -> base_link 的当前位姿。失败返回 false（pose 不修改）。
    bool lookupPose(nav_common::Pose2d& pose);

    /// 规划：查当前位姿 + goal，调 planPath，把结果存进 path_ 并发布。
    void plan();

    /// Foxglove 3D 点击目标回调：保存 map 系点并触发一次规划。
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    /// 控制循环：查位姿 -> 纯追踪 -> 发布 cmd_vel；到达目标则停。
    void tick();

    // ---- 已给全 ----
    void costMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void publishPath();
    void publishZeroCmdVel();

    // ---- 状态 ----
    nav_common::GridMap costmap_;
    bool has_costmap_ = false;
    std::vector<nav_common::Vec2d> path_;
    bool has_goal_ = false;
    nav_common::Vec2d goal_{0.0, 0.0};

    // ---- 参数 ----
    std::string cost_map_topic_;
    std::string goal_topic_;
    std::string cmd_vel_topic_;
    std::string path_topic_;
    double control_rate_ = 20.0;
    double goal_tolerance_ = 0.15;
    double lookahead_distance_ = 0.5;
    double max_linear_velocity_ = 1.0;
    double slowdown_distance_ = 1.0;
    double max_linear_acceleration_ = 1.0;
    double max_linear_deceleration_ = 1.5;
    double max_angular_velocity_ = 2.0;
    int occupied_threshold_ = 99;
    double previous_linear_velocity_ = 0.0;

    // ---- ROS 对象 ----
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr path_publish_timer_;
};

} // namespace nav_executor
