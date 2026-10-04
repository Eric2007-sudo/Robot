// locator_common::tf_utils 的单元测试。
//
// 这些测试检查的是函数的**契约**，不是实现：给定输入应该得到什么输出。
// 下发给学生，运行方式见 README。
//
//   colcon test --packages-select locator_common
//   colcon test-result --verbose
//
// 测试全过不代表整条链路就对了，只说明这四个工具函数的行为符合约定。

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <locator_common/tf_utils.hpp>

namespace tf_utils = locator_common::tf_utils;

namespace {

constexpr double EPS = 1e-9;

void expect_vec_near(const tf2::Vector3& got, const tf2::Vector3& want, double eps = EPS) {
    EXPECT_NEAR(got.x(), want.x(), eps);
    EXPECT_NEAR(got.y(), want.y(), eps);
    EXPECT_NEAR(got.z(), want.z(), eps);
}

} // namespace

// ============================================================================
// make_transform
// ============================================================================

TEST(MakeTransform, KeepsTranslation) {
    const tf2::Transform tf = tf_utils::make_transform(
        tf2::Vector3(1.0, 2.0, 3.0), 0.0, 0.0, 0.0
    );
    expect_vec_near(tf.getOrigin(), tf2::Vector3(1.0, 2.0, 3.0));
}

TEST(MakeTransform, ZeroAnglesGiveIdentityRotation) {
    const tf2::Transform tf =
        tf_utils::make_transform(tf2::Vector3(0.0, 0.0, 0.0), 0.0, 0.0, 0.0);
    expect_vec_near(tf * tf2::Vector3(1.0, 0.0, 0.0), tf2::Vector3(1.0, 0.0, 0.0));
    expect_vec_near(tf * tf2::Vector3(0.0, 1.0, 0.0), tf2::Vector3(0.0, 1.0, 0.0));
}

// yaw 绕 +z 转：转 90° 后，x 轴应该指向原来的 +y
TEST(MakeTransform, YawRotatesAboutZ) {
    const tf2::Transform tf = tf_utils::make_transform(
        tf2::Vector3(0.0, 0.0, 0.0), 0.0, 0.0, M_PI_2
    );
    expect_vec_near(tf * tf2::Vector3(1.0, 0.0, 0.0), tf2::Vector3(0.0, 1.0, 0.0));
    expect_vec_near(tf * tf2::Vector3(0.0, 0.0, 1.0), tf2::Vector3(0.0, 0.0, 1.0));
}

// pitch 绕 +y 转：按右手定则，转 90° 后 +x 指向 -z
TEST(MakeTransform, PitchRotatesAboutY) {
    const tf2::Transform tf = tf_utils::make_transform(
        tf2::Vector3(0.0, 0.0, 0.0), 0.0, M_PI_2, 0.0
    );
    expect_vec_near(tf * tf2::Vector3(1.0, 0.0, 0.0), tf2::Vector3(0.0, 0.0, -1.0));
    expect_vec_near(tf * tf2::Vector3(0.0, 1.0, 0.0), tf2::Vector3(0.0, 1.0, 0.0));
}

// roll 绕 +x 转：转 90° 后 +y 指向 +z
TEST(MakeTransform, RollRotatesAboutX) {
    const tf2::Transform tf = tf_utils::make_transform(
        tf2::Vector3(0.0, 0.0, 0.0), M_PI_2, 0.0, 0.0
    );
    expect_vec_near(tf * tf2::Vector3(0.0, 1.0, 0.0), tf2::Vector3(0.0, 0.0, 1.0));
    expect_vec_near(tf * tf2::Vector3(1.0, 0.0, 0.0), tf2::Vector3(1.0, 0.0, 0.0));
}

// 三个角同时非零时，必须与 tf2::Quaternion::setRPY 完全一致。
// 这一条能挡住"用了另一个欧拉角约定"的写法。
TEST(MakeTransform, MatchesSetRPY) {
    const double roll = 0.3, pitch = -0.7, yaw = 1.1;
    tf2::Quaternion want;
    want.setRPY(roll, pitch, yaw);

    const tf2::Transform got = tf_utils::make_transform(
        tf2::Vector3(0.0, 0.0, 0.0), roll, pitch, yaw
    );
    EXPECT_NEAR(got.getRotation().angleShortestPath(want), 0.0, 1e-9);
}

// 平移不参与旋转：变换作用在点上时，先转再平移
TEST(MakeTransform, TranslationAppliedAfterRotation) {
    const tf2::Transform tf = tf_utils::make_transform(
        tf2::Vector3(5.0, 0.0, 0.0), 0.0, 0.0, M_PI_2
    );
    expect_vec_near(tf * tf2::Vector3(1.0, 0.0, 0.0), tf2::Vector3(5.0, 1.0, 0.0));
}

// ============================================================================
// camera_optical_rotation
// ============================================================================

// 光学系与机械系原点重合，只差一个旋转
TEST(CameraOpticalRotation, HasNoTranslation) {
    expect_vec_near(
        tf_utils::camera_optical_rotation().getOrigin(), tf2::Vector3(0.0, 0.0, 0.0)
    );
}

// 必须是正交的右手系旋转：行列式为 +1。写成左手系（某个轴符号错了）会在这里被挡住。
TEST(CameraOpticalRotation, IsProperRotation) {
    const tf2::Matrix3x3 basis = tf_utils::camera_optical_rotation().getBasis();
    EXPECT_NEAR(basis.determinant(), 1.0, 1e-9);

    const tf2::Matrix3x3 should_be_identity = basis * basis.transpose();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            EXPECT_NEAR(should_be_identity[i][j], i == j ? 1.0 : 0.0, 1e-9);
        }
    }
}

// 光轴正前方 2 m 处的一个点，在 camera_link 下应当是正前方 2 m。
// 这里用物理含义表述，没有直接给出矩阵：camera_link 遵循 REP-103（x 前 y 左 z 上），
// camera_optical 遵循光学系约定。两者的轴怎么对应，自己推。
TEST(CameraOpticalRotation, PointAheadOnOpticalAxisIsAheadInCameraLink) {
    const tf2::Transform optical_to_link = tf_utils::camera_optical_rotation();
    const tf2::Vector3 in_link = optical_to_link * tf2::Vector3(0.0, 0.0, 2.0);
    expect_vec_near(in_link, tf2::Vector3(2.0, 0.0, 0.0));
}

TEST(CameraOpticalRotation, PointToTheRightOfImageIsToTheRightOfVehicle) {
    const tf2::Transform optical_to_link = tf_utils::camera_optical_rotation();
    // 光学系 +x 是像面向右
    const tf2::Vector3 in_link = optical_to_link * tf2::Vector3(1.0, 0.0, 0.0);
    // REP-103 下 +y 是左，所以向右是 -y
    expect_vec_near(in_link, tf2::Vector3(0.0, -1.0, 0.0));
}

TEST(CameraOpticalRotation, PointBelowImageCenterIsBelowInCameraLink) {
    const tf2::Transform optical_to_link = tf_utils::camera_optical_rotation();
    // 光学系 +y 是像面向下
    const tf2::Vector3 in_link = optical_to_link * tf2::Vector3(0.0, 1.0, 0.0);
    // REP-103 下 +z 是上，所以向下是 -z
    expect_vec_near(in_link, tf2::Vector3(0.0, 0.0, -1.0));
}

// ============================================================================
// to_transform_stamped
// ============================================================================

TEST(ToTransformStamped, FillsHeaderAndFrames) {
    const tf2::Transform tf = tf_utils::make_transform(
        tf2::Vector3(1.0, 2.0, 3.0), 0.0, 0.0, 0.0
    );
    const rclcpp::Time stamp(42, 123456789, RCL_ROS_TIME);

    const auto msg = tf_utils::to_transform_stamped(tf, "parent", "child", stamp);

    EXPECT_EQ(msg.header.frame_id, "parent");
    EXPECT_EQ(msg.child_frame_id, "child");
    EXPECT_EQ(msg.header.stamp.sec, 42);
    EXPECT_EQ(msg.header.stamp.nanosec, 123456789u);
    EXPECT_NEAR(msg.transform.translation.x, 1.0, EPS);
    EXPECT_NEAR(msg.transform.translation.y, 2.0, EPS);
    EXPECT_NEAR(msg.transform.translation.z, 3.0, EPS);
}

TEST(ToTransformStamped, PreservesRotation) {
    tf2::Quaternion q;
    q.setRPY(0.1, 0.2, 0.3);
    const tf2::Transform tf(q, tf2::Vector3(0.0, 0.0, 0.0));

    const auto msg =
        tf_utils::to_transform_stamped(tf, "a", "b", rclcpp::Time(0, 0, RCL_ROS_TIME));

    // 空消息的 rotation 四个分量全是 0，不是单位四元数，这里能区分出来
    EXPECT_NEAR(
        msg.transform.rotation.x * msg.transform.rotation.x
            + msg.transform.rotation.y * msg.transform.rotation.y
            + msg.transform.rotation.z * msg.transform.rotation.z
            + msg.transform.rotation.w * msg.transform.rotation.w,
        1.0,
        1e-9
    );

    tf2::Quaternion got;
    tf2::fromMsg(msg.transform.rotation, got);
    EXPECT_NEAR(got.angleShortestPath(q), 0.0, 1e-9);
}

// ============================================================================
// to_pose
// ============================================================================

TEST(ToPose, CopiesTranslationAndRotation) {
    const tf2::Transform tf = tf_utils::make_transform(
        tf2::Vector3(-1.5, 0.5, 2.5), 0.0, 0.4, 0.0
    );
    const auto pose = tf_utils::to_pose(tf);

    EXPECT_NEAR(pose.position.x, -1.5, EPS);
    EXPECT_NEAR(pose.position.y, 0.5, EPS);
    EXPECT_NEAR(pose.position.z, 2.5, EPS);

    tf2::Quaternion got;
    tf2::fromMsg(pose.orientation, got);
    EXPECT_NEAR(got.angleShortestPath(tf.getRotation()), 0.0, 1e-9);
}

// ============================================================================
// try_lookup_tf
// ============================================================================

class TryLookupTf: public ::testing::Test {
protected:
    void SetUp() override {
        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
        err_count_ = 0;
        last_err_.clear();
    }

    /// 往 buffer 里塞一条 parent -> child 的变换
    void set_tf(
        const std::string& parent,
        const std::string& child,
        const tf2::Transform& tf,
        const rclcpp::Time& stamp
    ) {
        buffer_->setTransform(
            tf_utils::to_transform_stamped(tf, parent, child, stamp), "unit_test"
        );
    }

    std::function<void(const std::string&)> recorder() {
        return [this](const std::string& err) {
            err_count_++;
            last_err_ = err;
        };
    }

    std::shared_ptr<rclcpp::Clock> clock_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    int err_count_ = 0;
    std::string last_err_;
};

TEST_F(TryLookupTf, ReturnsTrueAndTransformsPoint) {
    const rclcpp::Time stamp(10, 0, RCL_ROS_TIME);
    // child 的原点在 parent 的 (1, 0, 0)，且绕 z 转了 90°
    set_tf(
        "parent",
        "child",
        tf_utils::make_transform(tf2::Vector3(1.0, 0.0, 0.0), 0.0, 0.0, M_PI_2),
        stamp
    );

    tf2::Transform out;
    const bool ok =
        tf_utils::try_lookup_tf(buffer_, "parent", "child", stamp, out, recorder());

    ASSERT_TRUE(ok);
    EXPECT_EQ(err_count_, 0);
    // child 系下的 (1, 0, 0) 这个点，在 parent 系下应当是 (1, 1, 0)
    expect_vec_near(out * tf2::Vector3(1.0, 0.0, 0.0), tf2::Vector3(1.0, 1.0, 0.0));
}

// target / source 的顺序必须和 tf2_ros::Buffer::lookupTransform 一致。
// 用一个不对称的变换，写反了结果会明显不同。
TEST_F(TryLookupTf, TargetSourceOrderMatchesTf2) {
    const rclcpp::Time stamp(10, 0, RCL_ROS_TIME);
    set_tf(
        "parent",
        "child",
        tf_utils::make_transform(tf2::Vector3(3.0, 0.0, 0.0), 0.0, 0.0, 0.0),
        stamp
    );

    tf2::Transform parent_from_child;
    ASSERT_TRUE(tf_utils::try_lookup_tf(
        buffer_, "parent", "child", stamp, parent_from_child, recorder()
    ));
    expect_vec_near(parent_from_child.getOrigin(), tf2::Vector3(3.0, 0.0, 0.0));

    tf2::Transform child_from_parent;
    ASSERT_TRUE(tf_utils::try_lookup_tf(
        buffer_, "child", "parent", stamp, child_from_parent, recorder()
    ));
    expect_vec_near(child_from_parent.getOrigin(), tf2::Vector3(-3.0, 0.0, 0.0));
}

// 多段链路应当自动串联
TEST_F(TryLookupTf, ComposesChain) {
    const rclcpp::Time stamp(10, 0, RCL_ROS_TIME);
    set_tf(
        "a",
        "b",
        tf_utils::make_transform(tf2::Vector3(1.0, 0.0, 0.0), 0.0, 0.0, M_PI_2),
        stamp
    );
    set_tf(
        "b",
        "c",
        tf_utils::make_transform(tf2::Vector3(2.0, 0.0, 0.0), 0.0, 0.0, 0.0),
        stamp
    );

    tf2::Transform a_from_c;
    ASSERT_TRUE(tf_utils::try_lookup_tf(buffer_, "a", "c", stamp, a_from_c, recorder()));
    // b 绕 z 转了 90°，所以 b 系下的 +x 平移在 a 系下是 +y 方向
    expect_vec_near(a_from_c.getOrigin(), tf2::Vector3(1.0, 2.0, 0.0));
}

// frame 不存在：返回 false，把错误信息交给回调，不抛异常
TEST_F(TryLookupTf, ReturnsFalseOnUnknownFrame) {
    tf2::Transform out;
    bool ok = true;
    EXPECT_NO_THROW({
        ok = tf_utils::try_lookup_tf(
            buffer_, "nope", "nothing", rclcpp::Time(10, 0, RCL_ROS_TIME), out, recorder()
        );
    });
    EXPECT_FALSE(ok);
    EXPECT_EQ(err_count_, 1);
    EXPECT_FALSE(last_err_.empty());
}

// 查询时刻超出已有数据范围：同样返回 false
TEST_F(TryLookupTf, ReturnsFalseWhenTimeOutOfRange) {
    set_tf(
        "parent",
        "child",
        tf2::Transform::getIdentity(),
        rclcpp::Time(10, 0, RCL_ROS_TIME)
    );

    tf2::Transform out;
    const bool ok = tf_utils::try_lookup_tf(
        buffer_, "parent", "child", rclcpp::Time(30, 0, RCL_ROS_TIME), out, recorder()
    );
    EXPECT_FALSE(ok);
    EXPECT_EQ(err_count_, 1);
}

// 数据没齐时必须立刻返回，不能阻塞等待。
// 这一条挡住"顺手给 lookupTransform 加个 timeout"的写法。
TEST_F(TryLookupTf, DoesNotBlockWhenDataMissing) {
    tf2::Transform out;
    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = tf_utils::try_lookup_tf(
        buffer_, "parent", "child", rclcpp::Time(10, 0, RCL_ROS_TIME), out, recorder()
    );
    const auto elapsed = std::chrono::steady_clock::now() - t0;

    EXPECT_FALSE(ok);
    EXPECT_LT(
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 50
    );
}

// 两个时刻之间的查询，tf2 会插值
TEST_F(TryLookupTf, InterpolatesBetweenTwoStamps) {
    set_tf(
        "parent",
        "child",
        tf_utils::make_transform(tf2::Vector3(0.0, 0.0, 0.0), 0.0, 0.0, 0.0),
        rclcpp::Time(10, 0, RCL_ROS_TIME)
    );
    set_tf(
        "parent",
        "child",
        tf_utils::make_transform(tf2::Vector3(10.0, 0.0, 0.0), 0.0, 0.0, 0.0),
        rclcpp::Time(20, 0, RCL_ROS_TIME)
    );

    tf2::Transform out;
    ASSERT_TRUE(tf_utils::try_lookup_tf(
        buffer_, "parent", "child", rclcpp::Time(15, 0, RCL_ROS_TIME), out, recorder()
    ));
    expect_vec_near(out.getOrigin(), tf2::Vector3(5.0, 0.0, 0.0), 1e-6);
}
