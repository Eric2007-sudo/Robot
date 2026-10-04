#pragma once

#include <string>

#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace locator_sim {

/// 仿真侧专用的消息转换。
///
/// 仿真节点是整套校验的"裁判"：它们发布的 TF 和真值必须不依赖被测代码。
/// 因此这里刻意不复用 locator_common::tf_utils，而是自己保留一份实现。
/// 否则被测侧的转换函数一旦写错，真值和解算结果会被同一个 bug 同时污染，
/// 逐点比对反而会显示"通过"。

/// tf2::Transform -> TransformStamped，parent_frame 是 header.frame_id
inline geometry_msgs::msg::TransformStamped make_transform_stamped(
    const tf2::Transform& tf,
    const std::string& parent_frame,
    const std::string& child_frame,
    const rclcpp::Time& stamp
) {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = parent_frame;
    msg.child_frame_id = child_frame;
    msg.transform = tf2::toMsg(tf);
    return msg;
}

/// tf2::Transform -> Pose
inline geometry_msgs::msg::Pose make_pose(const tf2::Transform& tf) {
    geometry_msgs::msg::Pose pose;
    tf2::toMsg(tf, pose);
    return pose;
}

} // namespace locator_sim
