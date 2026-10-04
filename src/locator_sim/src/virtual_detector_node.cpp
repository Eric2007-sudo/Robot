#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <locator_common/frames.hpp>
#include <locator_sim/msg_conversions.hpp>
#include <locator_sim/scene_model.hpp>
#include <locator_sim/scene_params_loader.hpp>
#include <locator_sim/sim_clock.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace locator_sim {
using namespace geometry_msgs::msg;
namespace frames = locator_common::frames;

/// 虚拟上游 detector 节点。
///
/// 模拟一个"看到目标并解出目标位姿"的视觉节点。真实 detector 的输出就是目标在**相机
/// 光学系**下的位姿（PnP 或深度相机的原生输出），所以这里也发在 camera_optical 下。
///
/// 按需求不做 IPPE 二义性筛选：直接规定相机系下看到的目标位姿唯一且正确。真实场景里
/// 这一步对应"已经完成了解的消歧"。
///
/// 两个真实流水线特征被保留：
///   1. **曝光时刻与发布时刻分离**。消息时间戳是曝光时刻，但消息在
///      曝光时刻 + pipeline_delay 才发出。这既符合真实的检测耗时，也是下游能按时间戳做
///      精确插值的前提：TF 插值需要一个晚于查询时刻的变换，若在曝光瞬间就发出去，
///      查询会跑在串口/IMU 的下一帧之前而失败。
///   2. **视场裁剪**。目标不在视场内就不发，下游要能容忍数据间断。
///
/// 输出：geometry_msgs/PoseStamped，header.frame_id = camera_optical
class VirtualDetectorNode: public rclcpp::Node {
public:
    explicit VirtualDetectorNode(const rclcpp::NodeOptions& options)
    : Node("virtual_detector", options) {
        scene_ = declare_scene_params(*this);

        const double publish_rate = declare_parameter<double>("detector.publish_rate");
        pipeline_delay_ = declare_parameter<double>("detector.pipeline_delay");
        half_fov_h_ = declare_parameter<double>("detector.half_fov_horizontal");
        half_fov_v_ = declare_parameter<double>("detector.half_fov_vertical");
        max_range_ = declare_parameter<double>("detector.max_range");
        const std::string target_topic = declare_parameter<std::string>("detector.target_topic");
        const std::string truth_topic = declare_parameter<std::string>("detector.truth_topic");

        target_pub_ = create_publisher<PoseStamped>(target_topic, rclcpp::QoS(10));
        // 真值旁路：仅供校验脚本使用，locator 不订阅它。
        // 与观测用同一个时间戳发出，校验时可按时间戳精确配对，无需知道仿真起始时刻。
        truth_pub_ = create_publisher<PoseStamped>(truth_topic, rclcpp::QoS(10));

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            [this]() { process_frame(); }
        );

        RCLCPP_INFO(
            get_logger(),
            "virtual detector up: %.1f Hz, pipeline delay %.1f ms, fov %.1f x %.1f deg",
            publish_rate,
            pipeline_delay_ * 1e3,
            half_fov_h_ * 2.0 * 180.0 / M_PI,
            half_fov_v_ * 2.0 * 180.0 / M_PI
        );
    }

private:
    /// 判断目标是否落在视场内。输入是光学系下的坐标：x 右、y 下、z 前。
    bool in_field_of_view(const tf2::Vector3& target_in_optical) const {
        // z <= 0 表示目标在相机背后，直接不可见
        if (target_in_optical.z() <= 1e-6) {
            return false;
        }
        if (target_in_optical.length() > max_range_) {
            return false;
        }
        const double ang_h = std::atan2(std::abs(target_in_optical.x()), target_in_optical.z());
        const double ang_v = std::atan2(std::abs(target_in_optical.y()), target_in_optical.z());
        return ang_h <= half_fov_h_ && ang_v <= half_fov_v_;
    }

    /// 处理一帧：在曝光时刻采样真值，算出目标在光学系下的位姿，延迟后发布
    void process_frame() {
        // 此刻定时器触发，代表这一帧的图像处理刚刚结束。图像的曝光发生在
        // pipeline_delay_ 之前，那才是这份观测真正对应的时刻。
        const rclcpp::Time now = get_clock()->now();
        const rclcpp::Time exposure_stamp = now - rclcpp::Duration::from_seconds(pipeline_delay_);
        const double t = SimClock::elapsed(get_clock(), exposure_stamp);

        const SceneState s = evaluate_scene(t, scene_);

        // 目标在 world 下的完整位姿
        const tf2::Transform target_to_world(s.target_orientation_in_world, s.target_in_world);
        // 变换到相机光学系：optical_to_world 的逆左乘
        const tf2::Transform target_to_optical = s.optical_to_world.inverse() * target_to_world;

        if (!in_field_of_view(target_to_optical.getOrigin())) {
            RCLCPP_DEBUG_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "target out of field of view, frame dropped"
            );
            return;
        }

        PoseStamped msg;
        msg.header.stamp = exposure_stamp;
        msg.header.frame_id = frames::CAMERA_OPTICAL;
        msg.pose = make_pose(target_to_optical);
        target_pub_->publish(msg);

        PoseStamped truth;
        truth.header.stamp = exposure_stamp;
        truth.header.frame_id = frames::WORLD;
        truth.pose = make_pose(target_to_world);
        truth_pub_->publish(truth);
    }

    SceneParams scene_;
    double pipeline_delay_ = 0.0;
    double half_fov_h_ = 0.0;
    double half_fov_v_ = 0.0;
    double max_range_ = 0.0;

    rclcpp::Publisher<PoseStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<PoseStamped>::SharedPtr truth_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace locator_sim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(locator_sim::VirtualDetectorNode)
