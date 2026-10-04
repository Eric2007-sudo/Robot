#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>

namespace locator_sim {

/// 整个仿真场景的解析真值模型。
///
/// 所有虚拟硬件节点（串口、IMU、detector）都从这一个头文件取真值，保证它们描述的是
/// 同一个世界；locator 只能看到这些节点发出的消息，不能看这里。最终校验就是比对
/// locator 重建出的目标世界坐标与 target_position() 的差。
///
/// 时间基准 t 是"仿真开始以来的秒数"，各节点用自己的时间戳算，因此必须共享同一个 t0。
struct SceneParams {
    // ---- 目标（飞行器）的卡西尼卵形线 ----
    /// 焦点半距，两焦点位于 world 的 (±a, 0)
    double cassini_a = 6.0;
    /// 卡西尼常数，曲线上任意点到两焦点距离之积恒为 b^2
    double cassini_b = 6.6;
    /// 卵形线绕行角速度（rad/s），对应参数角 theta 的推进速度
    double cassini_omega = 0.35;
    /// 曲线所在平面的基准高度
    double target_height = 8.0;
    /// 高度起伏幅值，让目标在竖直方向也有运动
    double target_height_amp = 1.5;

    // ---- 底盘在 world 下的运动 ----
    /// 底盘绕原点转圈的半径
    double chassis_radius = 12.0;
    /// 底盘绕行角速度
    double chassis_omega = 0.25;
    /// 底盘质心高度
    double chassis_height = 0.30;
    /// 颠簸：pitch 幅值与频率
    double bump_pitch_amp = 0.10;
    double bump_pitch_freq = 7.5;
    /// 颠簸：roll 幅值与频率
    double bump_roll_amp = 0.08;
    double bump_roll_freq = 9.0;
    /// 颠簸：竖直方向跳动幅值与频率
    double bump_z_amp = 0.04;
    double bump_z_freq = 11.0;

    // ---- 云台与相机的固连安装量 ----
    /// yaw 关节原点在 base_link 下的位置
    double gimbal_yaw_offset_x = 0.0;
    double gimbal_yaw_offset_y = 0.0;
    double gimbal_yaw_offset_z = 0.35;
    /// pitch 关节原点在 gimbal_yaw 下的位置
    double gimbal_pitch_offset_x = 0.05;
    double gimbal_pitch_offset_y = 0.0;
    double gimbal_pitch_offset_z = 0.12;
    /// 相机在 gimbal_pitch 下的位置（完全固定）
    double camera_offset_x = 0.10;
    double camera_offset_y = 0.0;
    double camera_offset_z = 0.06;
    /// 相机相对 pitch 连杆的安装倾角（rad），模拟镜头不完全平行于炮管
    double camera_mount_roll = 0.0;
    double camera_mount_pitch = 0.0;
    double camera_mount_yaw = 0.0;

    // ---- 云台随动 ----
    /// 云台 pitch 的机械限位（rad），对称上下。
    /// 默认场景下实际需求约 1.27 rad（目标近乎过顶时），留一点余量使随动不被截断。
    double gimbal_pitch_limit = 1.45;
};

/// 卡西尼卵形线的极坐标解算。
///
/// 定义：到两焦点 (±a, 0) 的距离之积等于 b^2。代入极坐标并解 r^2 的二次方程得
///
///     r(theta) = a * sqrt( cos(2 theta) + sqrt( (b/a)^4 - sin^2(2 theta) ) )
///
/// b/a > 1 时为单连通的花生形闭曲线（本项目取 1.1）；b/a == 1 退化成伯努利双纽线；
/// b/a < 1 会裂成两个互不相连的环，theta 有禁区，不适合连续绕行。
inline double cassini_radius(const double theta, const double a, const double b) {
    const double ratio = b / a;
    const double ratio4 = ratio * ratio * ratio * ratio;
    const double sin2 = std::sin(2.0 * theta);
    // b/a > 1 时 ratio4 - sin2^2 恒大于 0，这里仍夹一次防止参数被配成退化值时出 NaN
    const double inner = std::max(ratio4 - sin2 * sin2, 0.0);
    const double outer = std::max(std::cos(2.0 * theta) + std::sqrt(inner), 0.0);
    return a * std::sqrt(outer);
}

/// 目标在 world 下的位置真值。xy 走卡西尼卵形线，z 叠一个二倍频起伏。
inline tf2::Vector3 target_position(const double t, const SceneParams& p) {
    const double theta = p.cassini_omega * t;
    const double r = cassini_radius(theta, p.cassini_a, p.cassini_b);
    return tf2::Vector3(
        r * std::cos(theta),
        r * std::sin(theta),
        p.target_height + p.target_height_amp * std::sin(2.0 * theta)
    );
}

/// 目标姿态真值。让机头指向自身速度方向（数值差分求速度），roll 恒零。
/// 目标自身姿态不参与定位链路的位置校验，只是让 detector 输出一个完整的 Pose。
inline tf2::Quaternion target_orientation(const double t, const SceneParams& p) {
    constexpr double DT = 1e-4;
    const tf2::Vector3 p0 = target_position(t - DT, p);
    const tf2::Vector3 p1 = target_position(t + DT, p);
    const tf2::Vector3 vel = p1 - p0;

    const double yaw = std::atan2(vel.y(), vel.x());
    const double horizontal = std::hypot(vel.x(), vel.y());
    // pitch 取负是因为 REP-103 下绕 +y 正向转动使机头朝下
    const double pitch = -std::atan2(vel.z(), horizontal);

    tf2::Quaternion q;
    q.setRPY(0.0, pitch, yaw);
    return q;
}

/// 底盘在 world 下的位姿真值：绕原点匀速转圈，车头沿切向，叠加三轴颠簸。
inline tf2::Transform chassis_pose(const double t, const SceneParams& p) {
    const double angle = p.chassis_omega * t;

    const tf2::Vector3 origin(
        p.chassis_radius * std::cos(angle),
        p.chassis_radius * std::sin(angle),
        p.chassis_height + p.bump_z_amp * std::sin(p.bump_z_freq * t)
    );

    // 车头朝切向，即绕行角 + 90°
    const double yaw = angle + M_PI_2;
    const double pitch = p.bump_pitch_amp * std::sin(p.bump_pitch_freq * t);
    const double roll = p.bump_roll_amp * std::cos(p.bump_roll_freq * t);

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::Transform(q, origin);
}

/// yaw 关节原点在 base_link 下的固连平移
inline tf2::Vector3 gimbal_yaw_offset(const SceneParams& p) {
    return tf2::Vector3(p.gimbal_yaw_offset_x, p.gimbal_yaw_offset_y, p.gimbal_yaw_offset_z);
}

/// pitch 关节原点在 gimbal_yaw 下的固连平移
inline tf2::Vector3 gimbal_pitch_offset(const SceneParams& p) {
    return tf2::Vector3(p.gimbal_pitch_offset_x, p.gimbal_pitch_offset_y, p.gimbal_pitch_offset_z);
}

/// 相机在 gimbal_pitch 下的位姿，完全固定
inline tf2::Transform camera_mount(const SceneParams& p) {
    tf2::Quaternion q;
    q.setRPY(p.camera_mount_roll, p.camera_mount_pitch, p.camera_mount_yaw);
    return tf2::Transform(
        q,
        tf2::Vector3(p.camera_offset_x, p.camera_offset_y, p.camera_offset_z)
    );
}

/// camera_link（REP-103：x 前、y 左、z 上）到 camera_optical（OpenCV：x 右、y 下、z 前）
/// 的固定旋转。两系原点重合，只差一个旋转。
///
/// 这里和 locator_common::tf_utils::camera_optical_rotation() 是同一个值，但刻意各留
/// 一份实现：仿真是校验的裁判，不能依赖被测代码。被测侧写错时，真值必须仍然是对的。
inline tf2::Transform camera_optical_rotation() {
    const tf2::Matrix3x3 basis(
        0.0, 0.0, 1.0,
        -1.0, 0.0, 0.0,
        0.0, -1.0, 0.0
    );
    return tf2::Transform(basis, tf2::Vector3(0.0, 0.0, 0.0));
}

/// 云台两轴随动解算（真值侧的正向运动学逆问题）。
///
/// 给定底盘位姿与目标世界坐标，求让相机光轴指向目标所需的 yaw / pitch 关节角。
/// 做法是把目标点变换到 yaw 关节系下，直接取方位角和俯仰角。这里忽略 pitch 关节到
/// 相机之间的小平移偏置，因此光轴并非严格穿过目标中心 —— 这正是真实云台的情形，
/// 目标只要落在视场内就够了，不影响定位链路的正确性。
inline void solve_gimbal_joints(
    const tf2::Transform& chassis_to_world,
    const tf2::Vector3& target_in_world,
    const SceneParams& p,
    double& yaw_out,
    double& pitch_out
) {
    // yaw 关节原点在 world 下的位置
    const tf2::Vector3 joint_in_world = chassis_to_world * gimbal_yaw_offset(p);
    // 目标相对该原点的方向，旋到 base_link 系下表达
    const tf2::Vector3 dir_in_base =
        chassis_to_world.getBasis().transpose() * (target_in_world - joint_in_world);

    yaw_out = std::atan2(dir_in_base.y(), dir_in_base.x());

    const double horizontal = std::hypot(dir_in_base.x(), dir_in_base.y());
    // 绕 +y 正转使光轴朝下，故目标在上方时 pitch 为负
    const double raw_pitch = -std::atan2(dir_in_base.z(), horizontal);
    pitch_out = std::clamp(raw_pitch, -p.gimbal_pitch_limit, p.gimbal_pitch_limit);
}

/// 云台 yaw 连杆在 base_link 下的位姿真值
inline tf2::Transform gimbal_yaw_pose(const double yaw, const SceneParams& p) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::Transform(q, gimbal_yaw_offset(p));
}

/// 云台 pitch 连杆在 gimbal_yaw 下的位姿真值
inline tf2::Transform gimbal_pitch_pose(const double pitch, const SceneParams& p) {
    tf2::Quaternion q;
    q.setRPY(0.0, pitch, 0.0);
    return tf2::Transform(q, gimbal_pitch_offset(p));
}

/// 一次性算出某时刻整条链路的全部真值，供各虚拟硬件节点取用
struct SceneState {
    double stamp_sec = 0.0;
    /// world -> base_link
    tf2::Transform chassis_to_world;
    /// base_link -> gimbal_yaw
    tf2::Transform yaw_to_chassis;
    /// gimbal_yaw -> gimbal_pitch
    tf2::Transform pitch_to_yaw;
    /// gimbal_pitch -> camera_link
    tf2::Transform camera_to_pitch;
    /// world -> camera_optical，链路累乘的结果
    tf2::Transform optical_to_world;
    /// 云台两个关节角
    double gimbal_yaw = 0.0;
    double gimbal_pitch = 0.0;
    /// 目标在 world 下的位姿
    tf2::Vector3 target_in_world;
    tf2::Quaternion target_orientation_in_world;
};

inline SceneState evaluate_scene(const double t, const SceneParams& p) {
    SceneState s;
    s.stamp_sec = t;
    s.chassis_to_world = chassis_pose(t, p);
    s.target_in_world = target_position(t, p);
    s.target_orientation_in_world = target_orientation(t, p);

    solve_gimbal_joints(s.chassis_to_world, s.target_in_world, p, s.gimbal_yaw, s.gimbal_pitch);

    s.yaw_to_chassis = gimbal_yaw_pose(s.gimbal_yaw, p);
    s.pitch_to_yaw = gimbal_pitch_pose(s.gimbal_pitch, p);
    s.camera_to_pitch = camera_mount(p);

    s.optical_to_world = s.chassis_to_world * s.yaw_to_chassis * s.pitch_to_yaw * s.camera_to_pitch
        * camera_optical_rotation();
    return s;
}

} // namespace locator_sim
