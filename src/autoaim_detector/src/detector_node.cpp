#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <autoaim_interfaces/msg/detections.hpp>
#include <autoaim_interfaces/msg/armor_detection.hpp>

#include "autoaim_detector/infer_engine.hpp"
#include "autoaim_detector/armor_refinator.hpp"
#include "autoaim_common_definitions/common_definitions.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoaim_detector {

class DetectorNode : public rclcpp::Node
{
public:
    explicit DetectorNode(const rclcpp::NodeOptions & options);

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

    // ===== 配置类（构造读一次，之后只读）=====
    bool enable_labeled_image_;
    bool enable_fps_;

    std::string input_image_topic_;
    std::string input_image_transport_;
    std::string labeled_image_topic_;
    std::string binary_image_topic_;
    std::string detections_topic_;

    std::string armor_model_name_;
    double armor_confidence_threshold_;
    double armor_keypoint_visibility_threshold_;
    double armor_nms_threshold_;

    int armor_refinator_brightness_thres_;
    int armor_refinator_color_diff_thres_;
    double armor_refinator_outward_width_ratio_;
    double armor_refinator_inward_tolerance_ratio_;
    double armor_refinator_vertical_margin_ratio_;
    double armor_refinator_min_color_ratio_;
    bool armor_refinator_filter_unlit_;

    std::vector<std::string> color_names_;
    std::vector<std::string> tag_names_;
    std::string model_path_;

    // ===== 运行时对象 =====
    std::unique_ptr<InferEngine> infer_engine_;
    std::unique_ptr<ArmorRefinator> armor_refinator_;

    image_transport::Subscriber image_sub_;
    rclcpp::Publisher<autoaim_interfaces::msg::Detections>::SharedPtr detections_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr labeled_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr binary_image_pub_;

    int frame_count_;
    rclcpp::Time last_fps_time_;
};

// 构造函数
DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
: Node("autoaim_detector", options)
{
    // 参数读取
    enable_labeled_image_ = declare_parameter<bool>("enable_labeled_image");
    enable_fps_ = declare_parameter<bool>("enable_fps");
    input_image_topic_ = declare_parameter<std::string>("input_image_topic");
    input_image_transport_ = declare_parameter<std::string>("input_image_transport");
    labeled_image_topic_ = declare_parameter<std::string>("labeled_image_topic");
    binary_image_topic_ = declare_parameter<std::string>("binary_image_topic");
    detections_topic_ = declare_parameter<std::string>("detections_topic");

    armor_model_name_ = declare_parameter<std::string>("armor_model_name");
    armor_confidence_threshold_ = declare_parameter<double>("armor_confidence_threshold");
    armor_keypoint_visibility_threshold_ = declare_parameter<double>("armor_keypoint_visibility_threshold");
    armor_nms_threshold_ = declare_parameter<double>("armor_nms_threshold");

    armor_refinator_brightness_thres_ = declare_parameter<int>("armor_refinator_brightness_thres");
    armor_refinator_color_diff_thres_ = declare_parameter<int>("armor_refinator_color_diff_thres");
    armor_refinator_outward_width_ratio_ = declare_parameter<double>("armor_refinator_outward_width_ratio");
    armor_refinator_inward_tolerance_ratio_ = declare_parameter<double>("armor_refinator_inward_tolerance_ratio");
    armor_refinator_vertical_margin_ratio_ = declare_parameter<double>("armor_refinator_vertical_margin_ratio");
    armor_refinator_min_color_ratio_ = declare_parameter<double>("armor_refinator_min_color_ratio");
    armor_refinator_filter_unlit_ = declare_parameter<bool>("armor_refinator_filter_unlit");

    color_names_ = {"Blue", "Red", "Gray"};
    tag_names_ = {"Sentry", "1", "2", "3", "4", "Outpost", "Base small", "Base big"};

    // 拼 model_path_ + 建 infer_engine_ / armor_refinator_
    const std::string share_dir = ament_index_cpp::get_package_share_directory("autoaim_detector");
    model_path_ = share_dir + "/models/" + armor_model_name_;
    infer_engine_ = create_infer_engine(
        model_path_,
        armor_model_contract::NUM_COLORS,
        armor_model_contract::NUM_CLASSES,
        armor_model_contract::NUM_KEYPOINTS,
        armor_model_contract::KEYPOINT_DIMS,
        static_cast<float>(armor_confidence_threshold_),
        static_cast<float>(armor_keypoint_visibility_threshold_),
        static_cast<float>(armor_nms_threshold_),
        color_names_,
        tag_names_,
        DetectionMode::ARMOR
    );
    armor_refinator_ = std::make_unique<ArmorRefinator>(
        armor_refinator_brightness_thres_,
        armor_refinator_color_diff_thres_,
        armor_refinator_outward_width_ratio_,
        armor_refinator_inward_tolerance_ratio_,
        armor_refinator_vertical_margin_ratio_,
        armor_refinator_min_color_ratio_,
        color_names_,
        tag_names_
    );

    // 建三个发布者
    detections_pub_ =
        create_publisher<autoaim_interfaces::msg::Detections>(detections_topic_, rclcpp::QoS(10));
    labeled_image_pub_ =
        create_publisher<sensor_msgs::msg::Image>(labeled_image_topic_, rclcpp::SensorDataQoS());
    binary_image_pub_ =
        create_publisher<sensor_msgs::msg::Image>(binary_image_topic_, rclcpp::SensorDataQoS());

    // 建订阅者 + INFO 日志
    image_sub_ = image_transport::create_subscription(
        this,
        input_image_topic_,
        std::bind(&DetectorNode::image_callback, this, std::placeholders::_1),
        input_image_transport_,
        rclcpp::SensorDataQoS().keep_last(1).get_rmw_qos_profile());

    frame_count_ = 0;
    last_fps_time_ = now();

    RCLCPP_INFO(
        get_logger(), "DetectorNode 初始化完成：订阅 %s (%s)",
        input_image_topic_.c_str(), input_image_transport_.c_str());
}

// 回调
void DetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
    // cv_bridge 转图 + 推理四步
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge 转换失败: %s", e.what());
        return;
    }
    const cv::Mat & frame = cv_ptr->image;
    infer_engine_->set_input_image(frame);
    infer_engine_->preprocess();
    infer_engine_->infer();
    infer_engine_->postprocess();

    // 灯条验证 + 组 Detections
    armor_refinator_->set_input_image(frame);
    const std::vector<DetectionObject> & detections =
        infer_engine_->get_detection_objects();

    autoaim_interfaces::msg::Detections out;
    out.header = msg->header;
    out.mode = static_cast<int>(DetectionMode::ARMOR);
    out.label = -1;
    out.armor_detections.reserve(detections.size());

    for (const auto & det : detections) {
        const bool lit = armor_refinator_->validate(det);
        if (armor_refinator_filter_unlit_ && !lit) {
            continue;
        }

        autoaim_interfaces::msg::ArmorDetection armor;
        armor.color = det.color;
        armor.label = det.label;
        armor.confidence = det.conf;

        armor.tl.x = det.kpts[0].pt.x; armor.tl.y = det.kpts[0].pt.y; armor.tl.z = 0.0f;
        armor.bl.x = det.kpts[1].pt.x; armor.bl.y = det.kpts[1].pt.y; armor.bl.z = 0.0f;
        armor.br.x = det.kpts[2].pt.x; armor.br.y = det.kpts[2].pt.y; armor.br.z = 0.0f;
        armor.tr.x = det.kpts[3].pt.x; armor.tr.y = det.kpts[3].pt.y; armor.tr.z = 0.0f;

        out.armor_detections.push_back(armor);
    }

    // 发布 + FPS
    detections_pub_->publish(out);

    if (enable_labeled_image_) {
        const cv::Mat labeled = infer_engine_->draw_labeled_image();
        if (!labeled.empty()) {
            labeled_image_pub_->publish(
                *cv_bridge::CvImage(msg->header, "bgr8", labeled).toImageMsg());
        }
    }

    const cv::Mat binary = armor_refinator_->draw_binary_image();
    if (!binary.empty()) {
        binary_image_pub_->publish(
            *cv_bridge::CvImage(msg->header, "mono8", binary).toImageMsg());
    }

    if (enable_fps_) {
        ++frame_count_;
        const rclcpp::Time now_time = now();
        const double elapsed = (now_time - last_fps_time_).seconds();
        if (elapsed >= 1.0) {
            RCLCPP_INFO(get_logger(), "处理帧率: %.1f FPS", frame_count_ / elapsed);
            frame_count_ = 0;
            last_fps_time_ = now_time;
        }
    }
}

}  // namespace autoaim_detector

RCLCPP_COMPONENTS_REGISTER_NODE(autoaim_detector::DetectorNode)
