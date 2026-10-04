#include <chrono>
#include <memory>
#include <string>

#include <locator_common/frames.hpp>
#include <locator_sim/msg_conversions.hpp>
#include <locator_sim/scene_model.hpp>
#include <locator_sim/scene_params_loader.hpp>
#include <locator_sim/sim_clock.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace locator_sim {
using namespace nav_msgs::msg;
namespace frames = locator_common::frames;

/// 虚拟串口节点。
///
/// 模拟一块从下位机（底盘控制板）读数据的串口驱动：按固定波特率节奏"收包"，每个包里带
/// 底盘在世界系下的位姿和一个时间戳，然后解包成 ROS 消息发出去。
///
/// 真实串口的两个特征这里都保留了：
///   1. 数据是下位机在**采样时刻**打的戳，不是上位机收到时刻。两者差一个传输延迟。
///      时间戳用采样时刻，否则下游按时间戳查 TF 会查到错误的姿态。
///   2. 位姿是下位机自己的里程计/定位结果，直接可用，不需要上位机再积分。
///
/// 输出：
///   - nav_msgs/Odometry  底盘在 world 下的位姿（twist 留空，本项目不需要）
///   - TF  world -> base_link
class VirtualSerialNode: public rclcpp::Node {
public:
    explicit VirtualSerialNode(const rclcpp::NodeOptions& options)
    : Node("virtual_serial", options) {
        scene_ = declare_scene_params(*this);

        const double publish_rate = declare_parameter<double>("serial.publish_rate");
        transport_delay_ = declare_parameter<double>("serial.transport_delay");
        const std::string odom_topic = declare_parameter<std::string>("serial.odom_topic");

        odom_pub_ = create_publisher<Odometry>(odom_topic, rclcpp::QoS(10));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            [this]() { publish_packet(); }
        );

        RCLCPP_INFO(
            get_logger(),
            "virtual serial up: %.1f Hz, transport delay %.1f ms",
            publish_rate,
            transport_delay_ * 1e3
        );
    }

private:
    /// 解一个"串口包"并发布
    void publish_packet() {
        // 上位机此刻收到包，但包内数据是 transport_delay_ 之前在下位机采样的。
        // 因此时间戳要回退，这才是这份位姿真正对应的时刻。
        const rclcpp::Time now = get_clock()->now();
        const rclcpp::Time sample_stamp = now - rclcpp::Duration::from_seconds(transport_delay_);
        const double t = SimClock::elapsed(get_clock(), sample_stamp);

        const tf2::Transform chassis_to_world = chassis_pose(t, scene_);

        tf_broadcaster_->sendTransform(make_transform_stamped(
            chassis_to_world,
            frames::WORLD,
            frames::BASE_LINK,
            sample_stamp
        ));

        Odometry odom;
        odom.header.stamp = sample_stamp;
        odom.header.frame_id = frames::WORLD;
        odom.child_frame_id = frames::BASE_LINK;
        odom.pose.pose = make_pose(chassis_to_world);
        odom_pub_->publish(odom);
    }

    SceneParams scene_;
    double transport_delay_ = 0.0;

    rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace locator_sim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(locator_sim::VirtualSerialNode)
