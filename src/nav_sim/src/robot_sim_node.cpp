#include "nav_sim/robot_sim_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <nav_common/frames.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace nav_sim {

namespace {
namespace frames = nav_common::frames;

tf2::Quaternion yawQuaternion(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return q;
}

geometry_msgs::msg::Pose tfToPose(const tf2::Transform& t) {
    geometry_msgs::msg::Pose p;
    p.position.x = t.getOrigin().x();
    p.position.y = t.getOrigin().y();
    p.position.z = 0.0;
    const tf2::Quaternion q = t.getRotation();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}
} // namespace

RobotSimNode::RobotSimNode(const rclcpp::NodeOptions& options)
: Node("robot_sim", options) {
    x_ = declare_parameter<double>("robot.start_x", 0.0);
    y_ = declare_parameter<double>("robot.start_y", 0.0);
    yaw_ = declare_parameter<double>("robot.start_yaw", 0.0);
    const double control_rate = declare_parameter<double>("robot.control_rate", 50.0);
    dt_ = 1.0 / control_rate;
    trajectory_sample_distance_ =
        declare_parameter<double>("robot.trajectory_sample_distance", 0.05);
    trajectory_max_points_ = static_cast<size_t>(std::max<int64_t>(
        1, declare_parameter<int>("robot.trajectory_max_points", 5000)));
    const double trajectory_publish_rate =
        declare_parameter<double>("robot.trajectory_publish_rate", 2.0);
    trajectory_publish_period_ticks_ = trajectory_publish_rate > 0.0
        ? std::max(1, static_cast<int>(std::lround(control_rate / trajectory_publish_rate)))
        : 0;

    map_to_odom_x_ = declare_parameter<double>("localization.map_to_odom_x", 0.0);
    map_to_odom_y_ = declare_parameter<double>("localization.map_to_odom_y", 0.0);
    map_to_odom_yaw_ = declare_parameter<double>("localization.map_to_odom_yaw", 0.0);

    const std::string odom_topic =
        declare_parameter<std::string>("robot.odom_topic", "/nav_sim/odom");
    const std::string truth_topic =
        declare_parameter<std::string>("robot.truth_topic", "/nav_sim/ground_truth");
    const std::string cmd_vel_topic =
        declare_parameter<std::string>("robot.cmd_vel_topic", "/nav_executor/cmd_vel");
    truth_path_topic_ = declare_parameter<std::string>(
        "robot.truth_path_topic", "/nav_sim/ground_truth_path");
    map_velocity_topic_ = declare_parameter<std::string>(
        "robot.map_velocity_topic", "/nav_sim/map_velocity");

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmdVelCallback(msg); });

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(10));
    truth_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(truth_topic, rclcpp::QoS(10));
    rclcpp::QoS path_qos(rclcpp::KeepLast(1));
    path_qos.transient_local();
    truth_path_pub_ = create_publisher<nav_msgs::msg::Path>(truth_path_topic_, path_qos);
    map_velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        map_velocity_topic_, rclcpp::QoS(10));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    // 定位结果：恒定 map -> odom。只发一次（static）。
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = get_clock()->now();
    map_to_odom.header.frame_id = frames::MAP;
    map_to_odom.child_frame_id = frames::ODOM;
    map_to_odom.transform.translation.x = map_to_odom_x_;
    map_to_odom.transform.translation.y = map_to_odom_y_;
    map_to_odom.transform.translation.z = 0.0;
    const tf2::Quaternion q_mo = yawQuaternion(map_to_odom_yaw_);
    map_to_odom.transform.rotation.x = q_mo.x();
    map_to_odom.transform.rotation.y = q_mo.y();
    map_to_odom.transform.rotation.z = q_mo.z();
    map_to_odom.transform.rotation.w = q_mo.w();
    static_tf_broadcaster_->sendTransform(map_to_odom);

    timer_ = create_wall_timer(
        std::chrono::duration<double>(dt_),
        [this]() { integrateAndPublish(); });

    RCLCPP_INFO(
        get_logger(),
        "robot sim up: %.1f Hz, start (%.2f, %.2f, %.2f), map->odom (%.2f, %.2f, %.2f)",
        control_rate, x_, y_, yaw_, map_to_odom_x_, map_to_odom_y_, map_to_odom_yaw_);
}

void RobotSimNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    v_ = msg->linear.x;
    w_ = msg->angular.z;
}

void RobotSimNode::integrateAndPublish() {
    // 差速运动学积分（欧拉法，dt 固定）
    x_ += v_ * std::cos(yaw_) * dt_;
    y_ += v_ * std::sin(yaw_) * dt_;
    yaw_ += w_ * dt_;

    // base_link 在 map 系下的位姿
    tf2::Transform T_map_base(yawQuaternion(yaw_), tf2::Vector3(x_, y_, 0.0));

    // 恒定的 map -> odom
    tf2::Transform T_map_odom(
        yawQuaternion(map_to_odom_yaw_), tf2::Vector3(map_to_odom_x_, map_to_odom_y_, 0.0));

    // odom -> base = (map -> odom)^-1 * (map -> base)
    const tf2::Transform T_odom_base = T_map_odom.inverse() * T_map_base;

    const rclcpp::Time now = get_clock()->now();

    // 动态 TF odom -> base_link
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = now;
    odom_tf.header.frame_id = frames::ODOM;
    odom_tf.child_frame_id = frames::BASE_LINK;
    odom_tf.transform.translation.x = T_odom_base.getOrigin().x();
    odom_tf.transform.translation.y = T_odom_base.getOrigin().y();
    odom_tf.transform.translation.z = 0.0;
    const tf2::Quaternion q_ob = T_odom_base.getRotation();
    odom_tf.transform.rotation.x = q_ob.x();
    odom_tf.transform.rotation.y = q_ob.y();
    odom_tf.transform.rotation.z = q_ob.z();
    odom_tf.transform.rotation.w = q_ob.w();
    tf_broadcaster_->sendTransform(odom_tf);

    // Odometry（odom 系）
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = frames::ODOM;
    odom.child_frame_id = frames::BASE_LINK;
    odom.pose.pose = tfToPose(T_odom_base);
    odom.twist.twist.linear.x = v_;
    odom.twist.twist.angular.z = w_;
    odom_pub_->publish(odom);

    // 实际速度转换到 map 系，供 Foxglove 观察；linear.y 是地图坐标的 y 方向速度，
    // 不是差速底盘的车体侧向速度。
    geometry_msgs::msg::TwistStamped map_velocity;
    map_velocity.header.stamp = now;
    map_velocity.header.frame_id = frames::MAP;
    map_velocity.twist.linear.x = v_ * std::cos(yaw_);
    map_velocity.twist.linear.y = v_ * std::sin(yaw_);
    map_velocity.twist.angular.z = w_;
    map_velocity_pub_->publish(map_velocity);

    // 真值旁路（map 系），仅校验脚本用
    geometry_msgs::msg::PoseStamped truth;
    truth.header.stamp = now;
    truth.header.frame_id = frames::MAP;
    truth.pose = tfToPose(T_map_base);
    truth_pub_->publish(truth);

    // Keep a bounded, distance-sampled history for visualization; ground_truth itself is only one pose.
    if (ground_truth_path_.empty() ||
        std::hypot(x_ - ground_truth_path_.back().pose.position.x,
                   y_ - ground_truth_path_.back().pose.position.y) >= trajectory_sample_distance_) {
        if (ground_truth_path_.size() >= trajectory_max_points_) ground_truth_path_.pop_front();
        ground_truth_path_.push_back(truth);
    }

    ++simulation_tick_;
    if (trajectory_publish_period_ticks_ > 0 &&
        simulation_tick_ % static_cast<uint64_t>(trajectory_publish_period_ticks_) == 0) {
        publishGroundTruthPath();
    }
}

void RobotSimNode::publishGroundTruthPath() {
    nav_msgs::msg::Path path;
    path.header.stamp = get_clock()->now();
    path.header.frame_id = frames::MAP;
    path.poses.assign(ground_truth_path_.begin(), ground_truth_path_.end());
    truth_path_pub_->publish(path);
}

} // namespace nav_sim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav_sim::RobotSimNode)
