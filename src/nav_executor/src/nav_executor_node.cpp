// 导航执行节点实现。
//
// 逻辑串起来是：代价图和目标就绪后，用 A* 规划一条世界系路径，发布出去；控制循环里
// 每个周期查一次当前位姿，用纯追踪算出 cmd_vel 发给底盘仿真。
//
// 你需要理解坐标系：map / odom / base_link 三层（见 docs/Part_of_Frames.md）。
// 目标、代价图、路径都在 map 系；车体当前位姿要靠查 TF（map -> base_link）得到，
// 不能直接拿 odom 系的里程计位姿当 map 系用。
#include "nav_executor/nav_executor_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <nav_common/frames.hpp>
#include <nav_controller/pure_pursuit.hpp>
#include <nav_planner/astar_planner.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace nav_executor {

namespace {
namespace frames = nav_common::frames;
}

NavExecutorNode::NavExecutorNode(const rclcpp::NodeOptions& options)
: Node("nav_executor", options) {
    // ---- 参数 ----
    cost_map_topic_ = declare_parameter<std::string>(
        "topics.cost_map_topic", "/map_server/global_cost_map");
    goal_topic_ = declare_parameter<std::string>("topics.goal_topic", "/nav_goal");
    cmd_vel_topic_ =
        declare_parameter<std::string>("topics.cmd_vel_topic", "/nav_executor/cmd_vel");
    path_topic_ =
        declare_parameter<std::string>("topics.path_topic", "/nav_executor/global_path");
    control_rate_ = declare_parameter<double>("control_rate", 20.0);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.15);
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.5);
    max_linear_velocity_ = declare_parameter<double>("max_linear_velocity", 1.0);
    slowdown_distance_ = declare_parameter<double>("slowdown_distance", 1.0);
    max_linear_acceleration_ = declare_parameter<double>("max_linear_acceleration", 1.0);
    max_linear_deceleration_ = declare_parameter<double>("max_linear_deceleration", 1.5);
    max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 2.0);
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 99);

    // ---- TF ----
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- 订阅 ----
    cost_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        cost_map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { costMapCallback(msg); });
    goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        goal_topic_, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) { goalCallback(msg); });

    // ---- 发布 ----
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    rclcpp::QoS path_qos(rclcpp::KeepLast(1));
    path_qos.transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, path_qos);

    // ---- 控制循环 ----
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_),
        [this]() { tick(); });

    // 计划路径在活动期间周期重发，晚连接的 Foxglove 也能看到。
    path_publish_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_),
        [this]() {
            if (!path_.empty()) publishPath();
        });

    RCLCPP_INFO(get_logger(), "nav_executor up (%.1f Hz)", control_rate_);
}

nav_common::GridMap NavExecutorNode::toGridMap(const nav_msgs::msg::OccupancyGrid& msg) {
    // TODO 1：把代价图消息原样转成 GridMap。字段一一对应，data 直接拷贝。
    (void)msg;
    return {};
}

bool NavExecutorNode::lookupPose(nav_common::Pose2d& pose) {
    // TODO 2：查 TF 拿 map -> base_link 的当前位姿。
    //   - 用 tf_buffer_->lookupTransform(frames::MAP, frames::BASE_LINK, tf2::TimePointZero)
    //     查最新一条（本仿真无时间戳延迟，用最新即可）。
    //   - 平移取 x/y；yaw 用 tf2::Quaternion + tf2::Matrix3x3 的 getRPY 提取。
    //   - 查询抛 tf2::TransformException 时 catch 掉并返回 false。
    (void)pose;
    return false;
}

void NavExecutorNode::plan() {
    // TODO 3：规划。步骤：
    //   1. 没有代价图或没有目标 -> 直接返回。
    //   2. lookupPose 拿当前位姿；查不到（TF 还没就绪）-> 返回，下个周期再试。
    //   3. 用 nav_planner::planPath(costmap_, {pose.x, pose.y}, goal_) 规划。
    //   4. 结果空（不可达）-> 打印警告、清空 path_ 并 publishPath() 发布空路径，
    //      清除 Foxglove 中之前的旧计划。
    //   5. 结果非空 -> 存进 path_，调 publishPath() 立即发布；活动期间定时器会重发。
}

void NavExecutorNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    // TODO 4：保存 Foxglove 发来的 map 系 PointStamped 目标，置 has_goal_，并触发一次 plan()。
    //   - 点坐标取 msg->point.x / msg->point.y；
    //   - 若 frame_id 非空且不是 map，应拒绝该目标，避免把其他坐标系误当成 map。
    (void)msg;
}

void NavExecutorNode::tick() {
    // TODO 5：控制循环。步骤：
    //   1. 没有目标 -> 发布零速（安全），返回。
    //   2. path_ 为空（还没规划出来）-> 尝试 plan()，然后仍发布零速，返回。
    //   3. lookupPose 拿当前位姿；查不到 -> 发布零速，返回。
    //   4. 到达判定：pose 到 goal_ 的距离 < goal_tolerance_ -> 发布零速，清空 path_，
    //      publishPath() 发布空路径清掉旧线，再清 has_goal_。
    //   5. 否则用 nav_controller::computeCommand(path_, pose, params) 算目标指令。
    //   6. 用上一周期线速度和 control_rate 限制加速/减速，再发布 Twist；不要让
    //      linear.x 在相邻周期突然跳变。
}

void NavExecutorNode::costMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    costmap_ = toGridMap(*msg);
    has_costmap_ = true;
    RCLCPP_INFO_ONCE(
        get_logger(), "cost map received: %dx%d, %.3f m/cell",
        costmap_.width, costmap_.height, costmap_.resolution);
}

void NavExecutorNode::publishPath() {
    nav_msgs::msg::Path out;
    out.header.stamp = get_clock()->now();
    out.header.frame_id = frames::MAP;
    out.poses.reserve(path_.size());
    for (const auto& p : path_) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = out.header;
        ps.pose.position.x = p.x;
        ps.pose.position.y = p.y;
        ps.pose.position.z = 0.0;
        ps.pose.orientation.w = 1.0;
        out.poses.push_back(ps);
    }
    path_pub_->publish(out);
}

void NavExecutorNode::publishZeroCmdVel() {
    geometry_msgs::msg::Twist t;
    t.linear.x = 0.0;
    t.angular.z = 0.0;
    cmd_vel_pub_->publish(t);
    previous_linear_velocity_ = 0.0;
}

} // namespace nav_executor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav_executor::NavExecutorNode)
