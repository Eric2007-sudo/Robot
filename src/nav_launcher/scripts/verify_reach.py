#!/usr/bin/env python3
"""闭环到达校验。

发一个目标点，看机器人（仿真真值）能否安全到达：整个过程中真值位姿不得进入障碍格，
且在超时时间内到达目标半径内。这检查的是规划 + 控制串起来之后的整条闭环。

用法：
    ros2 run nav_launcher verify_reach.py
    ros2 run nav_launcher verify_reach.py --ros-args -p goal_x:=4.0 -p goal_y:=-1.0
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class ReachVerifier(Node):
    def __init__(self):
        super().__init__('verify_reach')
        self.goal_x = self.declare_parameter('goal_x', 20.18).value
        self.goal_y = self.declare_parameter('goal_y', 9.98).value
        self.arrival_radius = self.declare_parameter('arrival_radius', 0.3).value
        self.timeout = self.declare_parameter('timeout', 120.0).value

        self.map_msg = None
        self.collided = False
        self.arrived = False
        self.min_dist = math.inf
        self.done = False

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map_server/global_cost_map', self.map_cb, latched)
        self.create_subscription(PoseStamped, '/nav_sim/ground_truth', self.truth_cb, 10)
        self.goal_pub = self.create_publisher(PointStamped, '/nav_goal', 10)

        self.sent = False
        self.started = self.get_clock().now()
        self.create_timer(0.5, self.tick)

    def map_cb(self, msg):
        self.map_msg = msg

    def truth_cb(self, msg):
        if self.map_msg is None:
            return
        x = msg.pose.position.x
        y = msg.pose.position.y
        d = math.hypot(x - self.goal_x, y - self.goal_y)
        self.min_dist = min(self.min_dist, d)
        if d < self.arrival_radius:
            self.arrived = True
        # 碰撞判定：真值点所在格是障碍（栅格规划把车当质点）
        c, r = self.world_to_grid(x, y)
        if self.is_occupied(c, r):
            self.collided = True

    def world_to_grid(self, x, y):
        info = self.map_msg.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        return int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))

    def is_occupied(self, c, r):
        info = self.map_msg.info
        w, h = info.width, info.height
        if not (0 <= c < w and 0 <= r < h):
            return True  # 出图视为碰撞
        return self.map_msg.data[r * w + c] >= 100

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
        if self.map_msg is None:
            return
        if not self.sent:
            self.send_goal()
            self.started = self.get_clock().now()
            return
        elapsed = (self.get_clock().now() - self.started).nanoseconds * 1e-9
        if self.arrived or self.collided or elapsed > self.timeout:
            self.report(elapsed)
            self.done = True

    def report(self, elapsed):
        print()
        print('=' * 66)
        print('  闭环到达校验结果')
        print('=' * 66)
        print(f'  目标点         : ({self.goal_x}, {self.goal_y})')
        print(f'  用时           : {elapsed:.1f} s')
        print(f'  最近距离       : {self.min_dist:.3f} m')

        if self.collided:
            print('  未通过：机器人碰撞障碍。')
            print('  （可能路径穿障、栅格换算错、或未正确查 TF 拿到 map 系位姿。）')
            self.ok = False
        elif self.arrived:
            print(f'  通过：安全到达目标半径 {self.arrival_radius} m 内。')
            self.ok = True
        else:
            print(f'  未通过：超时（{self.timeout:.0f} s）未到达。')
            print('  （可能没跑起来、规划不可达、或控制参数不合理。）')
            self.ok = False
        print('=' * 66)


def main():
    rclpy.init()
    node = ReachVerifier()
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
