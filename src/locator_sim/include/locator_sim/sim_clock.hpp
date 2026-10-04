#pragma once

#include <cstdint>

#include <rclcpp/rclcpp.hpp>

namespace locator_sim {

/// 仿真时间基准。
///
/// 各虚拟硬件节点跑在同一个进程（同一个 component container）里，但各自有独立的定时器。
/// 它们必须对"仿真开始的时刻"取得一致，否则算出的真值属于不同的世界，整条链路就没有
/// 可校验的闭合性。这里用一个进程内共享的静态 t0 来对齐：谁先跑谁负责初始化。
///
/// 用墙上时钟而不是 /clock + use_sim_time，是为了让时间戳对齐问题在真实时钟下暴露，
/// 而不是被仿真时钟掩盖。
class SimClock {
public:
    /// 取得（必要时初始化）共享的仿真起始时刻
    static rclcpp::Time epoch(const rclcpp::Clock::SharedPtr& clock) {
        static bool initialized = false;
        static rclcpp::Time t0(0, 0, RCL_ROS_TIME);
        if (!initialized) {
            t0 = clock->now();
            initialized = true;
        }
        return t0;
    }

    /// 把绝对时刻换算成"仿真开始以来的秒数"，即 scene_model 里的 t
    static double elapsed(const rclcpp::Clock::SharedPtr& clock, const rclcpp::Time& stamp) {
        return (stamp - epoch(clock)).seconds();
    }
};

} // namespace locator_sim
