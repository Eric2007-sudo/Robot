// 差速底盘仿真节点。订阅导航节点发来的 cmd_vel，用差速运动学积分出车体位姿，
// 发布里程计、TF 和真值旁路。只读，不要改；它同时是校验的裁判。
//
// 输出：
//   - 静态 TF        map -> odom             （定位结果，恒定，参数可配）
//   - 动态 TF        odom -> base_link       （里程计，control_rate Hz）
//   - Odometry       /nav_sim/odom           （frame=odom, child=base_link）
//   - PoseStamped    /nav_sim/ground_truth   （frame=map，当前真值点，校验脚本用）
//   - Path           /nav_sim/ground_truth_path（frame=map，距离采样的实际行驶轨迹，供可视化）
//   - TwistStamped   /nav_sim/map_velocity    （frame=map，实际地图系 x/y 速度，供可视化）
//
// 关键设计：定位结果（map->odom）是一个非平凡的恒定变换。车体在 map 系下的位姿，
// 必须通过查 TF 得到；直接把 Odometry 里的位姿当 map 系用会整体偏移。
#pragma once

#include <deque>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace nav_sim {

class RobotSimNode : public rclcpp::Node {
public:
    explicit RobotSimNode(const rclcpp::NodeOptions& options);

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void integrateAndPublish();
    void publishGroundTruthPath();

    // 内部状态：base_link 在 map 系下的真值位姿
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    // 当前速度指令
    double v_ = 0.0;
    double w_ = 0.0;

    // map -> odom 恒定变换
    double map_to_odom_x_ = 0.0;
    double map_to_odom_y_ = 0.0;
    double map_to_odom_yaw_ = 0.0;

    std::string truth_path_topic_;
    std::string map_velocity_topic_;
    double dt_ = 0.02;  // 1 / control_rate
    double trajectory_sample_distance_ = 0.05;
    size_t trajectory_max_points_ = 5000;
    int trajectory_publish_period_ticks_ = 25;
    uint64_t simulation_tick_ = 0;
    std::deque<geometry_msgs::msg::PoseStamped> ground_truth_path_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr truth_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr truth_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr map_velocity_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace nav_sim
