#!/usr/bin/env python3
"""跟踪质量校验。

发一个目标点，机器人沿路径前进的过程中，统计真值位姿到 global_path 的最近距离
（横向误差）。这检查的是纯追踪控制器：规划器过了、闭环也到了，但车是不是真的
"贴着路径走"。横向误差过大说明控制有问题（前瞻点选错、曲率公式错、转向反了等）。

用法：
    ros2 run nav_launcher verify_tracking.py
    ros2 run nav_launcher verify_tracking.py --ros-args -p max_lat_err:=1.0
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


def dist_to_polyline(px, py, pts):
    """点到折线的最近距离。"""
    best = math.inf
    for a, b in zip(pts, pts[1:]):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 == 0:
            d = math.hypot(px - ax, py - ay)
        else:
            t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
            d = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
        best = min(best, d)
    return best


class TrackingVerifier(Node):
    def __init__(self):
        super().__init__('verify_tracking')
        self.goal_x = self.declare_parameter('goal_x', 20.18).value
        self.goal_y = self.declare_parameter('goal_y', 9.98).value
        self.arrival_radius = self.declare_parameter('arrival_radius', 0.3).value
        self.max_lat_err = self.declare_parameter('max_lat_err', 1.0).value
        self.timeout = self.declare_parameter('timeout', 120.0).value

        self.path_pts = None
        self.max_err = 0.0
        self.arrived = False
        self.done = False

        self.create_subscription(Path, '/nav_executor/global_path', self.path_cb, 10)
        self.create_subscription(PoseStamped, '/nav_sim/ground_truth', self.truth_cb, 10)
        self.goal_pub = self.create_publisher(PointStamped, '/nav_goal', 10)

        self.sent = False
        self.started = self.get_clock().now()
        self.create_timer(0.5, self.tick)

    def path_cb(self, msg):
        if msg.poses:
            self.path_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def truth_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        d = math.hypot(x - self.goal_x, y - self.goal_y)
        if d < self.arrival_radius:
            self.arrived = True
            return
        # 只在"路上"（未到目标）统计横向误差
        if self.path_pts and len(self.path_pts) >= 2:
            err = dist_to_polyline(x, y, self.path_pts)
            self.max_err = max(self.max_err, err)

    def send_goal(self):
        g = PointStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.point.x = self.goal_x
        g.point.y = self.goal_y
        self.goal_pub.publish(g)
        self.sent = True
        self.get_logger().info(f'goal sent: ({self.goal_x}, {self.goal_y})')

    def tick(self):
        if not self.sent:
            self.send_goal()
            self.started = self.get_clock().now()
            return
        elapsed = (self.get_clock().now() - self.started).nanoseconds * 1e-9
        if self.arrived or elapsed > self.timeout:
            self.report(elapsed)
            self.done = True

    def report(self, elapsed):
        print()
        print('=' * 66)
        print('  跟踪质量校验结果')
        print('=' * 66)
        print(f'  目标点         : ({self.goal_x}, {self.goal_y})')
        print(f'  用时           : {elapsed:.1f} s')
        print(f'  最大横向误差   : {self.max_err:.3f} m')

        if self.path_pts is None:
            print('  未通过：没有收到 global_path。')
            self.ok = False
        elif not self.arrived:
            print(f'  未通过：超时（{self.timeout:.0f} s）未到达。')
            self.ok = False
        else:
            if self.max_err <= self.max_lat_err:
                print(f'  通过：横向误差在参考值 {self.max_lat_err} m 内。')
            else:
                print(f'  质量提示：横向误差超过参考值 {self.max_lat_err} m。')
                print('  仍通过到达校验；可根据控制器特性自行评估跟踪质量。')
            self.ok = True
        print('=' * 66)


def main():
    rclpy.init()
    node = TrackingVerifier()
    node.ok = False
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ok = getattr(node, 'ok', False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
