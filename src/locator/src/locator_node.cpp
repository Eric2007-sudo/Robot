#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <locator_common/frames.hpp>
#include <locator_common/tf_utils.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/subscription_options.hpp>

namespace locator {
using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;
using namespace visualization_msgs::msg;
namespace frames = locator_common::frames;
namespace tf_utils = locator_common::tf_utils;

/// 定位节点：把层层坐标系串成一条链，最终把上游给的相机系观测抬到世界系。
///
/// 输入（都是标准消息，无自定义接口）：
///   - sensor_msgs/Imu          IMU 固连在云台 pitch 连杆上，orientation 是该连杆
///                              相对 world 的**绝对姿态**，不是关节角
///   - geometry_msgs/PoseStamped 目标在相机光学系下的位姿
///   - TF world -> base_link    虚拟串口发布的底盘位姿
///
/// 输出：
///   - TF base_link -> gimbal_yaw -> gimbal_pitch   由 IMU 反解出的云台两轴
///   - 静态 TF gimbal_pitch -> camera_link -> camera_optical
///   - geometry_msgs/PoseStamped  目标在 world 下的位姿
///   - visualization_msgs/Marker  可视化（已给）
///
/// 各输入的话题、QoS、frame_id 与时间戳语义见 src/locator/docs/README.md。
class LocatorNode: public rclcpp::Node {
public:
    explicit LocatorNode(const rclcpp::NodeOptions& options)
    : Node("locator", options) {
        // ---- 固连安装量：物理装配决定，与仿真侧的 scene.* 必须一致 ----
        gimbal_yaw_offset_ = tf2::Vector3(
            declare_parameter<double>("mount.gimbal_yaw_offset_x"),
            declare_parameter<double>("mount.gimbal_yaw_offset_y"),
            declare_parameter<double>("mount.gimbal_yaw_offset_z")
        );
        gimbal_pitch_offset_ = tf2::Vector3(
            declare_parameter<double>("mount.gimbal_pitch_offset_x"),
            declare_parameter<double>("mount.gimbal_pitch_offset_y"),
            declare_parameter<double>("mount.gimbal_pitch_offset_z")
        );
        camera_to_pitch_ = tf_utils::make_transform(
            tf2::Vector3(
                declare_parameter<double>("mount.camera_offset_x"),
                declare_parameter<double>("mount.camera_offset_y"),
                declare_parameter<double>("mount.camera_offset_z")
            ),
            declare_parameter<double>("mount.camera_mount_roll"),
            declare_parameter<double>("mount.camera_mount_pitch"),
            declare_parameter<double>("mount.camera_mount_yaw")
        );

        enable_publish_marker_ = declare_parameter<bool>("enable_publish_marker");

        const std::string imu_topic = declare_parameter<std::string>("imu_topic");
        const std::string target_topic = declare_parameter<std::string>("target_topic");
        const std::string world_target_topic =
            declare_parameter<std::string>("world_target_topic");
        const std::string marker_topic = declare_parameter<std::string>("marker_topic");

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        // spin_thread 默认为 true，会起专用线程灌 /tf，带 timeout 的查询才有意义
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        publish_static_transforms();

        world_target_pub_ =
            create_publisher<PoseStamped>(world_target_topic, rclcpp::QoS(10));
        marker_pub_ = create_publisher<Marker>(marker_topic, rclcpp::QoS(10));

        // ====================================================================
        // TODO: 创建 IMU 与目标观测两个订阅。
        //
        // 要求：
        //   - IMU 回调与目标回调分别放进各自独立的回调组
        //   - 订阅的 QoS 要与发布端兼容（发布端 QoS 见 src/locator/docs/README.md）
        //   - 回调分别转发给下面的 imu_callback / target_callback
        //
        // 回调组这一条运行结果上不一定看得出来，但批改会看。为什么要这样做、
        // 它需要什么样的执行器配合，见 docs/Part_of_ROS2.md。
        // ====================================================================
        imu_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        target_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions imu_options;
        imu_options.callback_group = imu_cb_group_;
        imu_sub_ = create_subscription<Imu>(
            imu_topic,
            rclcpp::SensorDataQoS().keep_last(50),
            [this](const Imu::SharedPtr msg) { imu_callback(msg); },
            imu_options
        );

        rclcpp::SubscriptionOptions target_options;
        target_options.callback_group = target_cb_group_;
        target_sub_ = create_subscription<PoseStamped>(
            target_topic,
            rclcpp::QoS(10),
            [this](const PoseStamped::SharedPtr msg) { target_callback(msg); },
            target_options
        );

        RCLCPP_INFO(get_logger(), "locator up");
    }

private:
    /// 发布 gimbal_pitch -> camera_link -> camera_optical 两段静态 TF。
    ///
    /// 相机相对云台的安装是物理固连的，构造时发一次即可。
    /// gimbal_pitch -> camera_link 的安装量 camera_to_pitch_ 已在构造函数里读好；
    /// camera_link -> camera_optical 用 tf_utils::camera_optical_rotation()。
    void publish_static_transforms() {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        const rclcpp::Time stamp = now();
        transforms.push_back(tf_utils::to_transform_stamped(
            camera_to_pitch_, frames::GIMBAL_PITCH, frames::CAMERA_LINK, stamp
        ));
        transforms.push_back(tf_utils::to_transform_stamped(
            tf_utils::camera_optical_rotation(), frames::CAMERA_LINK, frames::CAMERA_OPTICAL,
            stamp
        ));
        static_broadcaster_->sendTransform(transforms);
    }

    /// 由 IMU 的绝对姿态反解云台 yaw / pitch 两个关节角，
    /// 并发布 base_link -> gimbal_yaw -> gimbal_pitch 两段 TF。
    ///
    /// 要求：
    ///   - 底盘姿态从 TF（world -> base_link）查，查询时刻用 IMU 消息自己的时间戳
    ///   - 不能阻塞等待 TF：数据没齐就丢掉这一帧，下一帧 2 ms 后就到
    ///   - 二轴云台没有 roll 关节。分解出的 roll 绝对值超过
    ///     residual_roll_warn_threshold_ 时，用 RCLCPP_WARN_THROTTLE 告警
    ///   - 发布的两段 TF 时间戳与 IMU 消息一致
    ///
    /// 关节原点的平移是 gimbal_yaw_offset_ / gimbal_pitch_offset_。
    void imu_callback(const Imu::SharedPtr msg) {
        const rclcpp::Time stamp = msg->header.stamp;

        tf2::Transform chassis_to_world;
        if (!tf_utils::try_lookup_tf(
                tf_buffer_, frames::WORLD, frames::BASE_LINK, stamp, chassis_to_world,
                [this](const std::string& err) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "chassis pose not available at imu stamp: %s", err.c_str());
                })) {
            return;
        }

        tf2::Quaternion imu_q;
        tf2::fromMsg(msg->orientation, imu_q);
        const tf2::Matrix3x3 R_world_pitch(imu_q);
        const tf2::Matrix3x3 R_world_base = chassis_to_world.getBasis();
        const tf2::Matrix3x3 R_base_pitch = R_world_base.transpose() * R_world_pitch;

        double yaw = 0.0, pitch = 0.0, roll = 0.0;
        R_base_pitch.getEulerYPR(yaw, pitch, roll);

        if (std::abs(roll) > residual_roll_warn_threshold_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "unexpected residual roll %.4f rad", roll);
        }

        const tf2::Transform yaw_to_base =
            tf_utils::make_transform(gimbal_yaw_offset_, 0.0, 0.0, yaw);
        const tf2::Transform pitch_to_yaw =
            tf_utils::make_transform(gimbal_pitch_offset_, 0.0, pitch, 0.0);

        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.push_back(tf_utils::to_transform_stamped(
            yaw_to_base, frames::BASE_LINK, frames::GIMBAL_YAW, stamp
        ));
        transforms.push_back(tf_utils::to_transform_stamped(
            pitch_to_yaw, frames::GIMBAL_YAW, frames::GIMBAL_PITCH, stamp
        ));
        tf_broadcaster_->sendTransform(transforms);
    }

    /// 把目标从相机光学系变换到世界系，发布到 world_target_pub_。
    ///
    /// 要求：
    ///   - 按观测的时间戳查询整条链路，链路的起点用消息自带的 frame_id，不要写死
    ///   - 同样不能阻塞等待 TF，查不到就丢掉这一帧
    ///   - 输出消息的时间戳必须与输入观测相同，frame_id 为 world。
    ///     校验脚本靠时间戳把你的输出和真值逐点配对
    ///   - enable_publish_marker_ 为 true 时调用 publish_marker
    void target_callback(const PoseStamped::SharedPtr msg) {
        const rclcpp::Time stamp = msg->header.stamp;

        tf2::Transform optical_to_world;
        if (!tf_utils::try_lookup_tf(
                tf_buffer_, frames::WORLD, msg->header.frame_id, stamp, optical_to_world,
                [&](const std::string& err) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "tf chain world -> %s not available at observation stamp: %s",
                        msg->header.frame_id.c_str(), err.c_str());
                })) {
            return;
        }

        tf2::Transform target_to_optical;
        tf2::fromMsg(msg->pose, target_to_optical);
        const tf2::Transform target_to_world = optical_to_world * target_to_optical;

        PoseStamped out;
        out.header.stamp = stamp;
        out.header.frame_id = frames::WORLD;
        out.pose = tf_utils::to_pose(target_to_world);
        world_target_pub_->publish(out);

        if (enable_publish_marker_) {
            publish_marker(target_to_world, stamp);
        }
    }

    // ------------------------------------------------------------------------
    // 以下可视化部分已给，不需要修改
    // ------------------------------------------------------------------------

    /// 把重建出的目标位置画成一个球和一条累积轨迹，在 Foxglove 里应当与真值卵形线重合
    void publish_marker(const tf2::Transform& target_to_world, const rclcpp::Time& stamp) {
        const tf2::Vector3 pos = target_to_world.getOrigin();

        Marker m;
        m.header.stamp = stamp;
        m.header.frame_id = frames::WORLD;
        m.ns = "located_target";
        m.id = 0;
        m.type = Marker::SPHERE;
        m.action = Marker::ADD;
        m.pose = tf_utils::to_pose(target_to_world);
        m.scale.x = 0.45;
        m.scale.y = 0.45;
        m.scale.z = 0.45;
        m.color.r = 0.20;
        m.color.g = 1.0;
        m.color.b = 0.40;
        m.color.a = 1.0;
        marker_pub_->publish(m);

        // 累积轨迹：这条线是最终的可视化产物，应当画出完整的卡西尼卵形线
        geometry_msgs::msg::Point p;
        p.x = pos.x();
        p.y = pos.y();
        p.z = pos.z();
        trail_points_.push_back(p);
        if (static_cast<int>(trail_points_.size()) > max_trail_points_) {
            trail_points_.erase(trail_points_.begin());
        }

        Marker trail;
        trail.header.stamp = stamp;
        trail.header.frame_id = frames::WORLD;
        trail.ns = "located_trail";
        trail.id = 0;
        trail.type = Marker::LINE_STRIP;
        trail.action = Marker::ADD;
        trail.pose.orientation.w = 1.0;
        trail.scale.x = 0.05;
        trail.color.r = 0.20;
        trail.color.g = 1.0;
        trail.color.b = 0.40;
        trail.color.a = 0.9;
        trail.points = trail_points_;
        marker_pub_->publish(trail);
    }

    // ---- 配置：构造时读一次 ----
    tf2::Vector3 gimbal_yaw_offset_ {0.0, 0.0, 0.0};
    tf2::Vector3 gimbal_pitch_offset_ {0.0, 0.0, 0.0};
    tf2::Transform camera_to_pitch_ = tf2::Transform::getIdentity();

    bool enable_publish_marker_ = false;
    /// 二轴云台分解出的残余 roll 超过这个值就报警（rad）
    double residual_roll_warn_threshold_ = 1e-3;
    int max_trail_points_ = 4000;
    std::vector<geometry_msgs::msg::Point> trail_points_;

    // ---- TF ----
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    // ---- 订阅与发布 ----
    rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
    rclcpp::CallbackGroup::SharedPtr target_cb_group_;
    rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<PoseStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<PoseStamped>::SharedPtr world_target_pub_;
    rclcpp::Publisher<Marker>::SharedPtr marker_pub_;
};

} // namespace locator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(locator::LocatorNode)
