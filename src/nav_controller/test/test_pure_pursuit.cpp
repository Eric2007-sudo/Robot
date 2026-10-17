// 纯追踪控制器单元测试。
//
// 运行：colcon test --packages-select nav_controller && colcon test-result --verbose
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <nav_common/pose.hpp>
#include <nav_controller/pure_pursuit.hpp>

using nav_common::Pose2d;
using nav_common::Vec2d;
using nav_controller::computeCommand;
using nav_controller::PurePursuitParams;

namespace {

PurePursuitParams params() {
    PurePursuitParams p;
    p.lookahead_distance = 0.5;
    p.max_linear_velocity = 1.0;
    p.max_angular_velocity = 3.0;
    p.goal_tolerance = 0.1;
    return p;
}

} // namespace

TEST(PurePursuit, EmptyPath) {
    auto cmd = computeCommand({}, Pose2d{0, 0, 0}, params());
    EXPECT_DOUBLE_EQ(cmd.v, 0.0);
    EXPECT_DOUBLE_EQ(cmd.w, 0.0);
}

TEST(PurePursuit, ProducesBoundedCommand) {
    std::vector<Vec2d> path{{0, 0}, {1, 0}, {2, 0}};
    Pose2d pose{0, 0, 0};  // 车在路径上，朝向路径
    auto cmd = computeCommand(path, pose, params());
    EXPECT_GE(cmd.v, 0.0);
    EXPECT_LE(cmd.v, params().max_linear_velocity);
    EXPECT_LE(std::abs(cmd.w), params().max_angular_velocity);
}

TEST(PurePursuit, SteersLeftWhenPathOnLeft) {
    // 路径沿 +y，车朝 +x（路径在车左侧）。ly > 0 -> w > 0（左转）。
    std::vector<Vec2d> path{{0, 0}, {0, 1}, {0, 2}};
    Pose2d pose{0, 0, 0};
    auto cmd = computeCommand(path, pose, params());
    EXPECT_GT(cmd.w, 0.0);
}

TEST(PurePursuit, StopsAtGoal) {
    std::vector<Vec2d> path{{0, 0}, {1, 0}};
    Pose2d pose{1, 0, 0};  // 已到终点
    auto cmd = computeCommand(path, pose, params());
    EXPECT_DOUBLE_EQ(cmd.v, 0.0);
    EXPECT_DOUBLE_EQ(cmd.w, 0.0);
}

TEST(PurePursuit, StopsNearGoal) {
    std::vector<Vec2d> path{{0, 0}, {1, 0}};
    Pose2d pose{0.95, 0, 0};  // 离终点 0.05 < goal_tolerance
    auto cmd = computeCommand(path, pose, params());
    EXPECT_DOUBLE_EQ(cmd.v, 0.0);
}

TEST(PurePursuit, SlowsNearGoal) {
    std::vector<Vec2d> path{{0, 0}, {2, 0}};
    const auto far = computeCommand(path, Pose2d{0, 0, 0}, params());
    const auto near = computeCommand(path, Pose2d{1.5, 0, 0}, params());
    EXPECT_GT(far.v, near.v);
    EXPECT_GT(near.v, 0.0);
}

TEST(PurePursuit, AngularVelocityClamped) {
    // 车朝向 +y，路径在 +x（车右侧）；角速度必须遵守配置的绝对上限。
    const double kPi = 3.14159265358979323846;
    std::vector<Vec2d> path{{0, 0}, {1, 0}, {2, 0}};
    Pose2d pose{0, 0, kPi / 2};
    auto p = params();
    p.max_angular_velocity = 0.5;
    auto cmd = computeCommand(path, pose, p);
    EXPECT_LT(cmd.w, 0.0);
    EXPECT_LE(std::abs(cmd.w), p.max_angular_velocity);
}
