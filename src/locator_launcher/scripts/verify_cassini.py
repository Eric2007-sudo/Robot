#!/usr/bin/env python3
"""从 locator 的输出验证卡西尼卵形线。

只订阅 locator/target_in_world，不看任何真值旁路，直接检验输出是否满足卡西尼卵形线的
定义：曲线上任意点到两焦点 (±a, 0) 的距离之积恒为 b^2。

这是"最终可视化是一条卡西尼卵形线"这个需求的数值证明。眼看 rviz 只能发现大错，
比这个乘积才能发现符号错和帧序错。

用法：
    ros2 run locator_launcher verify_cassini.py
    ros2 run locator_launcher verify_cassini.py --ros-args -p duration:=30.0
"""

import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile


class CassiniVerifier(Node):
    def __init__(self):
        super().__init__('cassini_verifier')

        # 这两个值必须与 locator_sim/config/sim_params.yaml 的 scene.cassini_* 一致
        self.a = self.declare_parameter('cassini_a', 6.0).value
        self.b = self.declare_parameter('cassini_b', 6.6).value
        self.duration = self.declare_parameter('duration', 25.0).value
        # 相对容差，乘积偏离 b^2 的百分比上限
        self.rel_tolerance = self.declare_parameter('relative_tolerance', 0.01).value
        topic = self.declare_parameter('topic', 'locator/target_in_world').value

        self.products = []
        self.points = []

        self.create_subscription(PoseStamped, topic, self.callback, QoSProfile(depth=200))
        self.start = self.get_clock().now()
        self.create_timer(0.5, self.tick)
        self.get_logger().info(
            f'verifying cassini oval a={self.a} b={self.b} for {self.duration:.1f} s'
        )

    def callback(self, msg):
        p = msg.pose.position
        d1 = math.hypot(p.x - self.a, p.y)
        d2 = math.hypot(p.x + self.a, p.y)
        self.products.append(d1 * d2)
        self.points.append((p.x, p.y, p.z))

    def tick(self):
        if (self.get_clock().now() - self.start).nanoseconds * 1e-9 < self.duration:
            return
        self.report()
        raise SystemExit

    def report(self):
        n = len(self.products)
        print()
        print('=' * 66)
        print('  卡西尼卵形线校验结果（仅用 locator 输出，不看真值）')
        print('=' * 66)

        if n == 0:
            print('  没有收到任何样本，检查 locator 是否在运行。')
            print('=' * 66)
            self.ok = False
            return

        target = self.b * self.b
        errors = [abs(v - target) for v in self.products]
        max_err = max(errors)
        rel_err = max_err / target * 100.0

        angles = sorted(math.atan2(y, x) for x, y, _ in self.points)
        gaps = [angles[i + 1] - angles[i] for i in range(len(angles) - 1)]
        max_gap = math.degrees(max(gaps)) if gaps else 360.0
        radii = [math.hypot(x, y) for x, y, _ in self.points]

        print(f'  样本数             : {n}')
        print(f'  目标 b^2           : {target:.6f}')
        print(f'  实测 d1*d2 均值    : {sum(self.products) / n:.6f}')
        print(f'  |d1*d2 - b^2| 最大 : {max_err:.3e}')
        print(f'  |d1*d2 - b^2| 平均 : {sum(errors) / n:.3e}')
        print(f'  相对误差最大       : {rel_err:.6f} %')
        print(f'  极角覆盖           : {math.degrees(angles[0]):+.1f}° .. '
              f'{math.degrees(angles[-1]):+.1f}°，最大间隙 {max_gap:.2f}°')
        print(f'  极径范围           : [{min(radii):.4f}, {max(radii):.4f}] m')

        print('-' * 66)
        closed = max_gap < 15.0
        accurate = rel_err <= self.rel_tolerance
        if accurate and closed:
            print(f'  通过：轨迹满足卡西尼卵形线定义，相对误差 {rel_err:.6f} % '
                  f'在容差 {self.rel_tolerance} % 内')
            print('  且极角覆盖完整，是一条闭合曲线。')
            self.ok = True
        else:
            if not accurate:
                print(f'  未通过：相对误差 {rel_err:.6f} % 超出容差 '
                      f'{self.rel_tolerance} %')
            if not closed:
                print(f'  未通过：极角最大间隙 {max_gap:.2f}° 过大，轨迹不完整')
            self.ok = False
        print('=' * 66)


def main():
    rclpy.init()
    node = CassiniVerifier()
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
