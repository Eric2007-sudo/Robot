#pragma once

#include <functional>
#include <memory>
#include <string>

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace locator_common::tf_utils {

// ============================================================================
// TODO: 实现下面五个函数。
//
// 签名和注释里的契约是给定的，不要改；locator_node.cpp 会按这个契约调用它们。
// 现在的函数体只是让工程能编过的占位，返回值都是错的。
// ============================================================================

/// 查询 TF：在 time 时刻，source 系到 target 系的变换。
///
/// 参数顺序与 tf2_ros::Buffer::lookupTransform 一致。返回的 tf_out 作用在
/// 一个 source 系下表达的点上，得到同一个点在 target 系下的表达。
///
/// 查询失败时不抛异常：把错误信息交给 err_info_handler，然后返回 false。
/// 成功时写入 tf_out 并返回 true。
///
/// 这个函数**刻意不提供 timeout 参数**，调用它不应该阻塞。为什么这样设计，
/// 见 docs/Part_of_TF.md。
inline bool try_lookup_tf(
    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
    const std::string& target,
    const std::string& source,
    const rclcpp::Time& time,
    tf2::Transform& tf_out,
    const std::function<void(const std::string& err_info)>& err_info_handler
) {
    // TODO
    (void)tf_buffer;
    (void)target;
    (void)source;
    (void)time;
    (void)tf_out;
    (void)err_info_handler;
    return false;
}

/// 由平移与 roll / pitch / yaw 三个角拼出一个变换。
///
/// 角度约定与 tf2::Quaternion::setRPY 一致：roll 绕 x、pitch 绕 y、yaw 绕 z，
/// 按 tf2 的固定轴约定组合。注意 tf2::Quaternion 里还有一个名字相近、约定不同的
/// 函数，别用错。
inline tf2::Transform make_transform(
    const tf2::Vector3& translation,
    const double roll,
    const double pitch,
    const double yaw
) {
    // TODO
    (void)translation;
    (void)roll;
    (void)pitch;
    (void)yaw;
    return tf2::Transform::getIdentity();
}

/// 把 tf2::Transform 填进一条 TransformStamped 消息。
///
/// 发布出去之后，这条消息描述的是 child_frame 相对 parent_frame 的位姿。
/// 想清楚 parent_frame / child_frame 分别对应消息里的哪个字段。
inline geometry_msgs::msg::TransformStamped to_transform_stamped(
    const tf2::Transform& tf,
    const std::string& parent_frame,
    const std::string& child_frame,
    const rclcpp::Time& stamp
) {
    // TODO
    (void)tf;
    (void)parent_frame;
    (void)child_frame;
    (void)stamp;
    return geometry_msgs::msg::TransformStamped();
}

/// 把 tf2::Transform 当作位姿，转成 geometry_msgs::msg::Pose。
inline geometry_msgs::msg::Pose to_pose(const tf2::Transform& tf) {
    // TODO
    (void)tf;
    return geometry_msgs::msg::Pose();
}

/// camera_link（REP-103：x 前 y 左 z 上）到 camera_optical（相机光学系）的固定旋转。
///
/// 两个坐标系原点重合，只差一个旋转，所以平移为零。
/// 两者的轴向约定、以及这个旋转怎么推出来，见 docs/Part_of_TF.md 和
/// docs/Part_of_Robotics_Math.md。
inline tf2::Transform camera_optical_rotation() {
    // TODO
    return tf2::Transform::getIdentity();
}

} // namespace locator_common::tf_utils
