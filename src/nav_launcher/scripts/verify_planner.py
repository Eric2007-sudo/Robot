#!/usr/bin/env python3
"""规划器校验。

只检查 nav_executor 发布的 global_path，不涉及机器人运动：给一个目标点，等导航节点
规划出一条路径，然后逐项核对：
  - 路径非空、首尾正确；
  - 路径点和连续路径段都不穿过障碍。

规划器可以使用不同的搜索、代价和路径平滑策略，不要求复现某一条参考路径。

用法：
    ros2 run nav_launcher verify_planner.py
    ros2 run nav_launcher verify_planner.py --ros-args -p goal_x:=4.0 -p goal_y:=-1.0
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def world_to_grid(info, x, y):
    res = info.resolution
    ox = info.origin.position.x
    oy = info.origin.position.y
    return int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))


class PlannerVerifier(Node):
    def __init__(self):
        super().__init__('verify_planner')
        self.goal_x = self.declare_parameter('goal_x', 20.18).value
        self.goal_y = self.declare_parameter('goal_y', 9.98).value
        self.map_msg = None
        self.path_msg = None
        self.truth = None

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map_server/global_cost_map', self.map_cb, latched)
        self.create_subscription(Path, '/nav_executor/global_path', self.path_cb, 10)
        self.create_subscription(PoseStamped, '/nav_sim/ground_truth', self.truth_cb, 10)
        self.goal_pub = self.create_publisher(PointStamped, '/nav_goal', 10)

        self.sent = False
        self.start_x = 0.0
        self.start_y = 0.0
        self.goal_sent_time = None
        self.done = False
        self.create_timer(0.5, self.tick)

    def map_cb(self, msg):
        self.map_msg = msg

    def path_cb(self, msg):
        if msg.poses:
            self.path_msg = msg

    def truth_cb(self, msg):
        self.truth = msg

    def send_goal(self):
        # 记录发目标瞬间的起点（之后车会动，不能再拿 truth 当起点）
        if self.truth is not None:
            self.start_x = self.truth.pose.position.x
            self.start_y = self.truth.pose.position.y
        g = PointStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.point.x = self.goal_x
        g.point.y = self.goal_y
        self.goal_pub.publish(g)
        self.sent = True
        self.goal_sent_time = self.get_clock().now()
        self.get_logger().info(f'goal sent: ({self.goal_x}, {self.goal_y})')

    def tick(self):
        if self.map_msg is None or self.truth is None:
            return
        if not self.sent:
            self.send_goal()
            return
        # 目标发出后等 1.5 秒，让 nav_executor 规划并发布 path（车还没走远）
        if (self.get_clock().now() - self.goal_sent_time).nanoseconds * 1e-9 < 1.5:
            return
        self.report()
        self.done = True

    def report(self):
        info = self.map_msg.info
        print()
        print('=' * 66)
        print('  规划器校验结果')
        print('=' * 66)
        if self.path_msg is None:
            print('  没有收到 global_path。检查 nav_executor 是否在运行、')
            print('  是否订阅了 /map_server/global_cost_map 与 /nav_goal。')
            print('=' * 66)
            self.ok = False
            return

        pts = [(p.pose.position.x, p.pose.position.y) for p in self.path_msg.poses]
        ok = True
        print(f'  目标点         : ({self.goal_x}, {self.goal_y})')
        print(f'  路径点数       : {len(pts)}')

        # 1. 首尾正确
        if not pts:
            print('  未通过：路径为空。')
            ok = False
        else:
            start_err = math.hypot(pts[0][0] - self.start_x,
                                   pts[0][1] - self.start_y)
            goal_err = math.hypot(pts[-1][0] - self.goal_x, pts[-1][1] - self.goal_y)
            print(f'  起点偏差       : {start_err:.3f} m')
            print(f'  终点偏差       : {goal_err:.3f} m')
            if start_err > info.resolution * 1.1 or goal_err > info.resolution * 1.1:
                print('  未通过：起点或终点不在正确格子里（栅格换算可能反了）。')
                ok = False

            # 2. 合法性
            if not self.path_valid(pts):
                print('  未通过：路径越界或路径点/线段进入障碍格。')
                ok = False
            else:
                print('  路径合法性     : 通过（路径点与连续线段均不穿障）')

            print('  路径长度       : 未作为通过条件（允许安全代价路径和平滑路径）')

        print('-' * 66)
        if ok:
            print('  通过：规划器输出的路径合法且可执行。')
        self.ok = ok
        print('=' * 66)

    def path_valid(self, pts):
        info = self.map_msg.info
        data = self.map_msg.data
        w, h = info.width, info.height

        def occ(c, r):
            return not (0 <= c < w and 0 <= r < h) or data[r * w + c] >= 99

        for (x, y) in pts:
            c, r = world_to_grid(info, x, y)
            if occ(c, r):
                return False
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            length = math.hypot(x1 - x0, y1 - y0)
            samples = max(1, math.ceil(length / (info.resolution * 0.5)))
            previous_col, previous_row = world_to_grid(info, x0, y0)
            for i in range(1, samples + 1):
                t = i / samples
                c, r = world_to_grid(
                    info, x0 + t * (x1 - x0), y0 + t * (y1 - y0))
                if (occ(c, r)
                        or (abs(c - previous_col) == 1 and abs(r - previous_row) == 1
                            and (occ(previous_col, r) or occ(c, previous_row)))):
                    return False
                previous_col, previous_row = c, r
        return True


def main():
    rclpy.init()
    node = PlannerVerifier()
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
