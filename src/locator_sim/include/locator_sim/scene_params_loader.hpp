#pragma once

#include <locator_sim/scene_model.hpp>
#include <rclcpp/rclcpp.hpp>

namespace locator_sim {

/// 从节点参数加载场景真值参数。
///
/// 三个虚拟硬件节点都调用这个函数，且共用同一份 YAML（`/**:` 通配），因此它们拿到的
/// 场景参数必然一致。参数不给默认值，YAML 里缺键就直接抛异常 —— 宁可启动失败，
/// 也不要用一个静默的默认值算出错误的真值。
inline SceneParams declare_scene_params(rclcpp::Node& node) {
    SceneParams p;

    p.cassini_a = node.declare_parameter<double>("scene.cassini_a");
    p.cassini_b = node.declare_parameter<double>("scene.cassini_b");
    p.cassini_omega = node.declare_parameter<double>("scene.cassini_omega");
    p.target_height = node.declare_parameter<double>("scene.target_height");
    p.target_height_amp = node.declare_parameter<double>("scene.target_height_amp");

    p.chassis_radius = node.declare_parameter<double>("scene.chassis_radius");
    p.chassis_omega = node.declare_parameter<double>("scene.chassis_omega");
    p.chassis_height = node.declare_parameter<double>("scene.chassis_height");
    p.bump_pitch_amp = node.declare_parameter<double>("scene.bump_pitch_amp");
    p.bump_pitch_freq = node.declare_parameter<double>("scene.bump_pitch_freq");
    p.bump_roll_amp = node.declare_parameter<double>("scene.bump_roll_amp");
    p.bump_roll_freq = node.declare_parameter<double>("scene.bump_roll_freq");
    p.bump_z_amp = node.declare_parameter<double>("scene.bump_z_amp");
    p.bump_z_freq = node.declare_parameter<double>("scene.bump_z_freq");

    p.gimbal_yaw_offset_x = node.declare_parameter<double>("scene.gimbal_yaw_offset_x");
    p.gimbal_yaw_offset_y = node.declare_parameter<double>("scene.gimbal_yaw_offset_y");
    p.gimbal_yaw_offset_z = node.declare_parameter<double>("scene.gimbal_yaw_offset_z");
    p.gimbal_pitch_offset_x = node.declare_parameter<double>("scene.gimbal_pitch_offset_x");
    p.gimbal_pitch_offset_y = node.declare_parameter<double>("scene.gimbal_pitch_offset_y");
    p.gimbal_pitch_offset_z = node.declare_parameter<double>("scene.gimbal_pitch_offset_z");
    p.camera_offset_x = node.declare_parameter<double>("scene.camera_offset_x");
    p.camera_offset_y = node.declare_parameter<double>("scene.camera_offset_y");
    p.camera_offset_z = node.declare_parameter<double>("scene.camera_offset_z");
    p.camera_mount_roll = node.declare_parameter<double>("scene.camera_mount_roll");
    p.camera_mount_pitch = node.declare_parameter<double>("scene.camera_mount_pitch");
    p.camera_mount_yaw = node.declare_parameter<double>("scene.camera_mount_yaw");

    p.gimbal_pitch_limit = node.declare_parameter<double>("scene.gimbal_pitch_limit");

    return p;
}

} // namespace locator_sim
