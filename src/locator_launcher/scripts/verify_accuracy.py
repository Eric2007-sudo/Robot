#!/usr/bin/env python3
"""定位精度校验。

对比 locator 重建出的目标世界坐标与仿真真值。两者按**时间戳精确配对**：detector 在同一
个曝光时刻同时发出观测（相机系）和真值（世界系），locator 处理观测后保持时间戳不变，
所以相同时间戳的两条消息描述的必然是同一时刻的同一个点。

无噪声仿真下整条链路是闭合的，误差应当在浮点精度量级。若误差显著大于 1 mm，说明
TF 链路存在实质错误（帧序颠倒、时间戳错位、安装量不一致等），而不是数值误差。

用法：
    ros2 run locator_launcher verify_accuracy.py
    ros2 run locator_launcher verify_accuracy.py --ros-args -p duration:=20.0
"""

import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile


class AccuracyVerifier(Node):
    def __init__(self):
        super().__init__('accuracy_verifier')

        self.duration = self.declare_parameter('duration', 25.0).value
        self.tolerance = self.declare_parameter('tolerance', 1e-3).value
        located_topic = self.declare_parameter(
            'located_topic', 'locator/target_in_world'
        ).value
        truth_topic = self.declare_parameter(
            'truth_topic', 'locator_sim/detector/target_truth'
        ).value

        qos = QoSProfile(depth=200)
        self.truths = {}
        self.located = {}
        self.errors = []

        self.create_subscription(PoseStamped, truth_topic, self.truth_cb, qos)
        self.create_subscription(PoseStamped, located_topic, self.located_cb, qos)

        self.start = self.get_clock().now()
        self.create_timer(0.5, self.tick)
        self.get_logger().info(
            f'collecting for {self.duration:.1f} s '
            f'(tolerance {self.tolerance * 1e3:.3f} mm)'
        )

    @staticmethod
    def key(msg):
        """时间戳作为配对键"""
        return (msg.header.stamp.sec, msg.header.stamp.nanosec)

    def truth_cb(self, msg):
        self.truths[self.key(msg)] = msg.pose.position
        self.match(self.key(msg))

    def located_cb(self, msg):
        self.located[self.key(msg)] = msg.pose.position
        self.match(self.key(msg))

    def match(self, k):
        if k in self.truths and k in self.located:
            a = self.truths.pop(k)
            b = self.located.pop(k)
            err = math.sqrt(
                (a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2
            )
            self.errors.append((err, a, b))

    def tick(self):
        elapsed = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        if elapsed < self.duration:
            return
        self.report()
        raise SystemExit

    def report(self):
        n = len(self.errors)
        print()
        print('=' * 66)
        print('  定位精度校验结果')
        print('=' * 66)

        if n == 0:
            print('  没有配对到任何样本。')
            print('  检查 locator 是否在运行，以及话题名是否匹配。')
            print('=' * 66)
            self.ok = False
            return

        errs = [e[0] for e in self.errors]
        errs.sort()
        max_err = errs[-1]
        mean_err = sum(errs) / n

        print(f'  配对样本数     : {n}')
        print(f'  平均误差       : {mean_err * 1e3:.6f} mm')
        print(f'  中位误差       : {errs[n // 2] * 1e3:.6f} mm')
        print(f'  最大误差       : {max_err * 1e3:.6f} mm')
        print(f'  未配对 (真值/解算): {len(self.truths)} / {len(self.located)}')

        # 覆盖范围：确认目标真的走了一整条闭合曲线，而不是只采到一小段
        xs = [e[1].x for e in self.errors]
        ys = [e[1].y for e in self.errors]
        zs = [e[1].z for e in self.errors]
        radii = [math.hypot(x, y) for x, y in zip(xs, ys)]
        print(f'  真值 x 范围    : [{min(xs):+.3f}, {max(xs):+.3f}] m')
        print(f'  真值 y 范围    : [{min(ys):+.3f}, {max(ys):+.3f}] m')
        print(f'  真值 z 范围    : [{min(zs):+.3f}, {max(zs):+.3f}] m')
        print(f'  真值极径范围   : [{min(radii):.3f}, {max(radii):.3f}] m')

        print('-' * 66)
        if max_err <= self.tolerance:
            print(f'  通过：最大误差 {max_err * 1e3:.6f} mm '
                  f'在容差 {self.tolerance * 1e3:.3f} mm 内')
            print('  整条 TF 链路闭合，重建的目标轨迹与真值一致。')
            self.ok = True
        else:
            print(f'  未通过：最大误差 {max_err * 1e3:.6f} mm '
                  f'超出容差 {self.tolerance * 1e3:.3f} mm')
            worst = max(self.errors, key=lambda e: e[0])
            print(f'  最差样本 真值=({worst[1].x:+.4f}, {worst[1].y:+.4f}, '
                  f'{worst[1].z:+.4f})')
            print(f'           解算=({worst[2].x:+.4f}, {worst[2].y:+.4f}, '
                  f'{worst[2].z:+.4f})')
            self.ok = False
        print('=' * 66)


def main():
    rclpy.init()
    node = AccuracyVerifier()
    node.ok = False
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        ok = getattr(node, 'ok', False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
