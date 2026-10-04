#include <chrono>
#include <memory>
#include <string>

#include <locator_common/frames.hpp>
#include <locator_sim/scene_model.hpp>
#include <locator_sim/scene_params_loader.hpp>
#include <locator_sim/sim_clock.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace locator_sim {
using namespace sensor_msgs::msg;
namespace frames = locator_common::frames;

/// 关节角真值消息里两个关节的名字，verify_gimbal.py 按这两个名字取值
inline const std::string JOINT_YAW = "gimbal_yaw_joint";
inline const std::string JOINT_PITCH = "gimbal_pitch_joint";

/// 虚拟 IMU 节点。
///
/// IMU 固连在**云台 pitch 连杆**上（和 RoboMaster 的实际装法一致），所以它测到的是
/// 云台末端在惯性系下的绝对姿态，而不是云台相对底盘的关节角。这一点是整个项目的关键：
/// 关节角必须由 locator 用 IMU 绝对姿态和底盘姿态做一次复合求逆才能得到。
///
/// 仿真保留真实 IMU 的语义，只去掉噪声和零偏：
///   - orientation：相对惯性系（world）的绝对姿态，来自内部姿态解算器的输出
///   - angular_velocity：陀螺原始量，表达在**自身坐标系**下，不是欧拉角速率
///   - linear_acceleration：加速度计测的是**比力**，含重力反作用。静止时读数为
///     机体系下的 +z 约 9.81，不是 0。这是加速度计的物理本质，不是 bug。
///
/// 输出：sensor_msgs/Imu，header.frame_id = gimbal_pitch
class VirtualImuNode: public rclcpp::Node {
public:
    explicit VirtualImuNode(const rclcpp::NodeOptions& options)
    : Node("virtual_imu", options) {
        scene_ = declare_scene_params(*this);

        const double publish_rate = declare_parameter<double>("imu.publish_rate");
        gravity_ = declare_parameter<double>("imu.gravity");
        transport_delay_ = declare_parameter<double>("imu.transport_delay");
        const std::string imu_topic = declare_parameter<std::string>("imu.topic");
        const std::string joint_truth_topic =
            declare_parameter<std::string>("imu.joint_truth_topic");

        imu_pub_ = create_publisher<Imu>(imu_topic, rclcpp::SensorDataQoS().keep_last(50));
        // 云台关节角真值旁路：仅供 verify_gimbal.py 使用，locator 不订阅。
        // 与 IMU 消息同一个时间戳发出，校验时可按时间戳精确配对。
        joint_truth_pub_ = create_publisher<JointState>(joint_truth_topic, rclcpp::QoS(50));

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            [this]() { publish_measurement(); }
        );

        RCLCPP_INFO(
            get_logger(),
            "virtual imu up: %.1f Hz on frame %s, transport delay %.1f ms",
            publish_rate,
            frames::GIMBAL_PITCH.c_str(),
            transport_delay_ * 1e3
        );
    }

private:
    /// IMU 所在连杆（云台 pitch 连杆）在 world 下的位姿真值
    tf2::Transform imu_pose_at(const double t) const {
        const SceneState s = evaluate_scene(t, scene_);
        return s.chassis_to_world * s.yaw_to_chassis * s.pitch_to_yaw;
    }

    /// 发布云台两个关节角的真值，时间戳与 IMU 消息一致
    void publish_joint_truth(const double t, const rclcpp::Time& stamp) {
        const SceneState s = evaluate_scene(t, scene_);

        JointState msg;
        msg.header.stamp = stamp;
        msg.name = {JOINT_YAW, JOINT_PITCH};
        msg.position = {s.gimbal_yaw, s.gimbal_pitch};
        joint_truth_pub_->publish(msg);
    }

    void publish_measurement() {
        // 数据在 transport_delay_ 之前采样，经 SPI/串口传到上位机才被这里读到。
        // 时间戳必须是采样时刻，这是下游按时间戳对齐姿态的前提。
        const rclcpp::Time now = get_clock()->now();
        const rclcpp::Time stamp = now - rclcpp::Duration::from_seconds(transport_delay_);
        const double t = SimClock::elapsed(get_clock(), stamp);

        // 中心差分的步长。太小会被 double 的舍入吃掉（差分是相减，有效位会掉），
        // 太大则跟不上颠簸的高频分量。1e-3 s 对最高 11 Hz 的颠簸足够。
        constexpr double DT = 1e-3;

        const tf2::Transform pose_prev = imu_pose_at(t - DT);
        const tf2::Transform pose_curr = imu_pose_at(t);
        const tf2::Transform pose_next = imu_pose_at(t + DT);

        Imu msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frames::GIMBAL_PITCH;

        // ---- 绝对姿态 ----
        msg.orientation = tf2::toMsg(pose_curr.getRotation());

        // ---- 陀螺：机体系角速度 ----
        // 由 R(t) 的导数得到。R_prev^T * R_next 是这 2*DT 内在机体系下的相对旋转，
        // 转成轴角后除以时间就是机体系角速度。直接对欧拉角求导是错的，
        // 因为欧拉角速率不等于角速度。
        const tf2::Matrix3x3 delta_rot =
            pose_prev.getBasis().transpose() * pose_next.getBasis();
        tf2::Quaternion delta_q;
        delta_rot.getRotation(delta_q);
        // 轴角：角度在 [0, pi]，小角度下数值稳定
        const double angle = delta_q.getAngle();
        const tf2::Vector3 axis = (angle > 1e-12) ? delta_q.getAxis() : tf2::Vector3(0, 0, 1);
        // getAngle 恒非负，但 getAxis 已带符号，两者组合即有向角速度
        const tf2::Vector3 omega_body = axis * (angle / (2.0 * DT));
        msg.angular_velocity.x = omega_body.x();
        msg.angular_velocity.y = omega_body.y();
        msg.angular_velocity.z = omega_body.z();

        // ---- 加速度计：比力 = (a_world - g_world) 旋到机体系 ----
        // 重力加速度向量在 world 下是 (0, 0, -g)。加速度计测不到重力本身，
        // 测的是支撑力对应的比力 f = a - g，所以静止时读数是 +z 方向的 g。
        const tf2::Vector3 accel_world =
            (pose_next.getOrigin() - 2.0 * pose_curr.getOrigin() + pose_prev.getOrigin())
            / (DT * DT);
        const tf2::Vector3 gravity_world(0.0, 0.0, -gravity_);
        const tf2::Vector3 specific_force =
            pose_curr.getBasis().transpose() * (accel_world - gravity_world);
        msg.linear_acceleration.x = specific_force.x();
        msg.linear_acceleration.y = specific_force.y();
        msg.linear_acceleration.z = specific_force.z();

        // 无噪声仿真，协方差填 0 表示"未知/不提供"以外的含义没有意义，
        // 这里给一个很小的正值，表示测量可信，便于下游滤波器直接用
        msg.orientation_covariance[0] = 1e-9;
        msg.orientation_covariance[4] = 1e-9;
        msg.orientation_covariance[8] = 1e-9;
        msg.angular_velocity_covariance[0] = 1e-9;
        msg.angular_velocity_covariance[4] = 1e-9;
        msg.angular_velocity_covariance[8] = 1e-9;
        msg.linear_acceleration_covariance[0] = 1e-9;
        msg.linear_acceleration_covariance[4] = 1e-9;
        msg.linear_acceleration_covariance[8] = 1e-9;

        imu_pub_->publish(msg);

        publish_joint_truth(t, stamp);
    }

    SceneParams scene_;
    double gravity_ = 9.81;
    double transport_delay_ = 0.0;

    rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<JointState>::SharedPtr joint_truth_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace locator_sim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(locator_sim::VirtualImuNode)
