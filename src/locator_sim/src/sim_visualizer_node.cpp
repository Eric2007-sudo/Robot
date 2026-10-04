#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <locator_common/frames.hpp>
#include <locator_sim/scene_model.hpp>
#include <locator_sim/scene_params_loader.hpp>
#include <locator_sim/sim_clock.hpp>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace locator_sim {
using namespace visualization_msgs::msg;
namespace frames = locator_common::frames;

/// 仿真可视化节点。
///
/// 只画"看得见的硬件"和真值参考：车体、云台两轴、相机、视场锥，以及卡西尼卵形线的
/// 解析真值曲线。这些 marker 挂在各自的 frame 上，由 TF 驱动运动，所以它们展示的是
/// TF 树是否正确 —— 如果 locator 解出的关节角错了，云台在 rviz 里就会指错方向。
///
/// 真值曲线用 world 系下的静态 LINE_STRIP 画出，作为 locator 重建结果的比对基准：
/// locator 发出的目标点应当正好落在这条线上。
class SimVisualizerNode: public rclcpp::Node {
public:
    explicit SimVisualizerNode(const rclcpp::NodeOptions& options)
    : Node("sim_visualizer", options) {
        scene_ = declare_scene_params(*this);

        const double publish_rate = declare_parameter<double>("visualizer.publish_rate");
        const std::string marker_topic = declare_parameter<std::string>("visualizer.marker_topic");
        trajectory_samples_ = declare_parameter<int>("visualizer.trajectory_samples");
        fov_range_ = declare_parameter<double>("visualizer.fov_range");
        fov_half_h_ = declare_parameter<double>("visualizer.fov_half_horizontal");
        fov_half_v_ = declare_parameter<double>("visualizer.fov_half_vertical");

        // 静态几何用 transient_local，rviz 后启动也能收到
        static_pub_ = create_publisher<MarkerArray>(
            marker_topic + "/static",
            rclcpp::QoS(1).transient_local()
        );
        dynamic_pub_ = create_publisher<MarkerArray>(marker_topic, rclcpp::QoS(10));

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            [this]() { publish_dynamic(); }
        );
        // 静态部分只发一次
        static_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() {
                publish_static();
                static_timer_->cancel();
            }
        );

        RCLCPP_INFO(get_logger(), "sim visualizer up: %.1f Hz", publish_rate);
    }

private:
    Marker make_marker(
        const std::string& frame,
        const std::string& ns,
        const int id,
        const int32_t type
    ) const {
        Marker m;
        m.header.frame_id = frame;
        m.header.stamp = get_clock()->now();
        m.ns = ns;
        m.id = id;
        m.type = type;
        m.action = Marker::ADD;
        // The geometry is published once, but its parent frame keeps moving.
        // Ask the visualization client to re-evaluate the marker with TF.
        m.frame_locked = frame != frames::WORLD;
        m.pose.orientation.w = 1.0;
        m.color.a = 1.0;
        return m;
    }

    /// 车体、云台、相机、视场锥，以及世界系下的卡西尼真值曲线
    void publish_static() {
        MarkerArray arr;
        int id = 0;

        // ---- 车体 ----
        {
            Marker m = make_marker(frames::BASE_LINK, "chassis", id++, Marker::CUBE);
            m.scale.x = 0.80;
            m.scale.y = 0.55;
            m.scale.z = 0.25;
            m.pose.position.z = 0.05;
            m.color.r = 0.14;
            m.color.g = 0.46;
            m.color.b = 0.99;
            m.color.a = 0.85;
            arr.markers.push_back(m);
        }
        // ---- 四个轮子 ----
        {
            const double wheel_x = 0.28;
            const double wheel_y = 0.32;
            int wheel_id = 0;
            for (const double sx : {-1.0, 1.0}) {
                for (const double sy : {-1.0, 1.0}) {
                    Marker m =
                        make_marker(frames::BASE_LINK, "wheels", wheel_id++, Marker::CYLINDER);
                    m.scale.x = 0.17;
                    m.scale.y = 0.17;
                    m.scale.z = 0.08;
                    m.pose.position.x = sx * wheel_x;
                    m.pose.position.y = sy * wheel_y;
                    m.pose.position.z = -0.10;
                    // 圆柱默认沿 z，绕 x 转 90° 让它沿 y 轴躺下
                    m.pose.orientation.x = std::sin(M_PI_4);
                    m.pose.orientation.w = std::cos(M_PI_4);
                    m.color.r = 0.13;
                    m.color.g = 0.13;
                    m.color.b = 0.13;
                    arr.markers.push_back(m);
                }
            }
        }
        // ---- 云台 yaw 连杆 ----
        {
            Marker m = make_marker(frames::GIMBAL_YAW, "gimbal", id++, Marker::CYLINDER);
            m.scale.x = 0.14;
            m.scale.y = 0.14;
            m.scale.z = 0.12;
            m.color.r = 0.13;
            m.color.g = 0.55;
            m.color.b = 0.13;
            arr.markers.push_back(m);
        }
        // ---- 云台 pitch 连杆，沿 +x 伸出代表炮管/指向 ----
        {
            Marker m = make_marker(frames::GIMBAL_PITCH, "gimbal", id++, Marker::CYLINDER);
            m.scale.x = 0.05;
            m.scale.y = 0.05;
            m.scale.z = 0.30;
            m.pose.position.x = 0.15;
            // 圆柱沿 z，绕 y 转 90° 让它沿 x 轴指向前方
            m.pose.orientation.y = std::sin(M_PI_4);
            m.pose.orientation.w = std::cos(M_PI_4);
            m.color.r = 0.55;
            m.color.g = 0.0;
            m.color.b = 0.0;
            arr.markers.push_back(m);
        }
        // ---- 相机 ----
        {
            Marker m = make_marker(frames::CAMERA_LINK, "camera", id++, Marker::CUBE);
            m.scale.x = 0.06;
            m.scale.y = 0.10;
            m.scale.z = 0.06;
            m.color.r = 0.35;
            m.color.g = 0.35;
            m.color.b = 0.35;
            arr.markers.push_back(m);
        }
        // ---- 视场锥，用光学系下的四条棱表示 ----
        {
            Marker m = make_marker(frames::CAMERA_OPTICAL, "fov", id++, Marker::LINE_LIST);
            m.scale.x = 0.02;
            m.color.r = 1.0;
            m.color.g = 0.85;
            m.color.b = 0.40;
            m.color.a = 0.55;
            const double z = fov_range_;
            const double x = z * std::tan(fov_half_h_);
            const double y = z * std::tan(fov_half_v_);
            geometry_msgs::msg::Point origin;
            const std::vector<std::pair<double, double>> corners {
                {x, y},
                {x, -y},
                {-x, -y},
                {-x, y}
            };
            std::vector<geometry_msgs::msg::Point> far_pts;
            for (const auto& [cx, cy] : corners) {
                geometry_msgs::msg::Point p;
                p.x = cx;
                p.y = cy;
                p.z = z;
                far_pts.push_back(p);
            }
            // 从光心到四个远角
            for (const auto& p : far_pts) {
                m.points.push_back(origin);
                m.points.push_back(p);
            }
            // 远平面四条边
            for (size_t i = 0; i < far_pts.size(); i++) {
                m.points.push_back(far_pts[i]);
                m.points.push_back(far_pts[(i + 1) % far_pts.size()]);
            }
            arr.markers.push_back(m);
        }
        // ---- 卡西尼卵形线真值曲线 ----
        {
            Marker m = make_marker(frames::WORLD, "truth_trajectory", id++, Marker::LINE_STRIP);
            m.scale.x = 0.06;
            m.color.r = 1.0;
            m.color.g = 0.30;
            m.color.b = 0.27;
            m.color.a = 0.65;
            // 覆盖完整一周：theta 从 0 走到 2pi
            const double period = 2.0 * M_PI / scene_.cassini_omega;
            for (int i = 0; i <= trajectory_samples_; i++) {
                const double t = period * static_cast<double>(i)
                    / static_cast<double>(trajectory_samples_);
                const tf2::Vector3 pos = target_position(t, scene_);
                geometry_msgs::msg::Point p;
                p.x = pos.x();
                p.y = pos.y();
                p.z = pos.z();
                m.points.push_back(p);
            }
            arr.markers.push_back(m);
        }
        // ---- 底盘行驶圆 ----
        {
            Marker m = make_marker(frames::WORLD, "chassis_path", id++, Marker::LINE_STRIP);
            m.scale.x = 0.04;
            m.color.r = 0.20;
            m.color.g = 0.50;
            m.color.b = 0.95;
            m.color.a = 0.40;
            for (int i = 0; i <= 180; i++) {
                const double angle = 2.0 * M_PI * static_cast<double>(i) / 180.0;
                geometry_msgs::msg::Point p;
                p.x = scene_.chassis_radius * std::cos(angle);
                p.y = scene_.chassis_radius * std::sin(angle);
                p.z = scene_.chassis_height;
                m.points.push_back(p);
            }
            arr.markers.push_back(m);
        }

        static_pub_->publish(arr);
        RCLCPP_INFO(get_logger(), "published %zu static markers", arr.markers.size());
    }

    /// 目标本体，以及相机光心到目标的视线
    void publish_dynamic() {
        const rclcpp::Time stamp = get_clock()->now();
        const double t = SimClock::elapsed(get_clock(), stamp);
        const SceneState s = evaluate_scene(t, scene_);

        MarkerArray arr;

        // 目标真值位置，画在 world 下
        {
            Marker m = make_marker(frames::WORLD, "target_truth", 0, Marker::SPHERE);
            m.scale.x = 0.55;
            m.scale.y = 0.55;
            m.scale.z = 0.55;
            m.pose.position.x = s.target_in_world.x();
            m.pose.position.y = s.target_in_world.y();
            m.pose.position.z = s.target_in_world.z();
            m.color.r = 0.87;
            m.color.g = 0.18;
            m.color.b = 0.16;
            m.color.a = 0.45;
            arr.markers.push_back(m);
        }
        dynamic_pub_->publish(arr);
    }

    SceneParams scene_;
    int trajectory_samples_ = 400;
    double fov_range_ = 6.0;
    double fov_half_h_ = 0.0;
    double fov_half_v_ = 0.0;

    rclcpp::Publisher<MarkerArray>::SharedPtr static_pub_;
    rclcpp::Publisher<MarkerArray>::SharedPtr dynamic_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr static_timer_;
};

} // namespace locator_sim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(locator_sim::SimVisualizerNode)
