#pragma once

#include <string>

namespace locator_common::frames {

/// TF 树的唯一命名来源。仿真侧和 locator 侧都从这里取，避免两边写错字符串对不上。
///
///   world                       惯性系，REP-103：x 前、y 左、z 上
///   └── base_link               车体系，虚拟串口发布（含颠簸带来的 pitch/roll）
///       └── gimbal_yaw          云台 yaw 关节，绕 base_link 的 +z 转
///           └── gimbal_pitch    云台 pitch 关节，绕 gimbal_yaw 的 +y 转
///               └── camera_link          相机机械安装系，REP-103，与 pitch 连杆固连
///                   └── camera_optical   相机光学系，OpenCV 约定：x 右、y 下、z 前
///
/// 除 camera_optical 外全部遵循 REP-103。detector 的观测量表达在 camera_optical 下，
/// 因为那是 PnP / 深度相机原生输出的坐标系。
inline const std::string WORLD = "world";
inline const std::string BASE_LINK = "base_link";
inline const std::string GIMBAL_YAW = "gimbal_yaw";
inline const std::string GIMBAL_PITCH = "gimbal_pitch";
inline const std::string CAMERA_LINK = "camera_link";
inline const std::string CAMERA_OPTICAL = "camera_optical";

/// 被定位的目标（飞行器）在 world 下的名字，仅用于可视化标注
inline const std::string TARGET = "target";

} // namespace locator_common::frames
