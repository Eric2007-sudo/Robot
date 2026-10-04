#!/usr/bin/env python3
"""云台关节角校验。

把 locator 发布的 base_link -> gimbal_yaw -> gimbal_pitch 两段 TF，与仿真给出的
关节角真值逐点比对。两者按**时间戳精确配对**：虚拟 IMU 在同一个采样时刻同时发出
IMU 消息和关节角真值，locator 处理 IMU 消息后保持时间戳不变，所以相同时间戳的
数据描述的必然是同一时刻的同一个状态。

它的用途是把定位误差拆开。verify_accuracy 只给出最终误差，看不出错在哪一步：

    本脚本通过、verify_accuracy 不通过  ->  关节角对了，问题在静态 TF 或 target 回调
    本脚本不通过                        ->  问题在 imu 回调（反解公式、时间戳、帧序）

只检查旋转，不检查平移。平移是固连安装量，直接来自参数，用
`ros2 run tf2_ros tf2_echo base_link gimbal_yaw` 看一眼即可。

用法：
    ros2 run locator_launcher verify_gimbal.py
    ros2 run locator_launcher verify_gimbal.py --ros-args -p duration:=20.0
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

JOINT_YAW = 'gimbal_yaw_joint'
JOINT_PITCH = 'gimbal_pitch_joint'


def quat_from_axis_angle(axis, angle):
    """绕单轴旋转的四元数，axis 取 'x' / 'y' / 'z'，返回 (x, y, z, w)"""
    h = angle / 2.0
    s, c = math.sin(h), math.cos(h)
    return {
        'x': (s, 0.0, 0.0, c),
        'y': (0.0, s, 0.0, c),
        'z': (0.0, 0.0, s, c),
    }[axis]


def angle_between(q1, q2):
    """两个姿态之间的旋转角（rad）。四元数的双重覆盖用 abs 处理"""
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    return 2.0 * math.acos(min(1.0, dot))


class GimbalVerifier(Node):
    def __init__(self):
        super().__init__('gimbal_verifier')

        self.duration = self.declare_parameter('duration', 25.0).value
        # 容差（rad）。默认参数下残差只来自底盘 TF 的插值弦误差，量级在 1e-5
        self.tolerance = self.declare_parameter('tolerance', 1e-3).value
        truth_topic = self.declare_parameter(
            'truth_topic', 'locator_sim/imu/gimbal_joint_truth'
        ).value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.yaw_frame = self.declare_parameter('yaw_frame', 'gimbal_yaw').value
        self.pitch_frame = self.declare_parameter('pitch_frame', 'gimbal_pitch').value

        qos = QoSProfile(depth=400)
        # 真值：时间戳 -> (yaw, pitch)
        self.truths = {}
        # locator 发的 TF：时间戳 -> 四元数
        self.yaw_tf = {}
        self.pitch_tf = {}
        self.yaw_errors = []
        self.pitch_errors = []
        self.yaw_done = set()
        self.pitch_done = set()
        # 父子写反时，会看到反向的那条边
        self.reversed_yaw = 0
        self.reversed_pitch = 0
        self.tf_msg_count = 0

        self.create_subscription(JointState, truth_topic, self.truth_cb, qos)
        self.create_subscription(TFMessage, '/tf', self.tf_cb, qos)

        self.start = self.get_clock().now()
        self.create_timer(0.5, self.tick)
        self.get_logger().info(
            f'collecting for {self.duration:.1f} s '
            f'(tolerance {math.degrees(self.tolerance):.4f} deg)'
        )

    @staticmethod
    def key(stamp):
        return (stamp.sec, stamp.nanosec)

    def truth_cb(self, msg):
        try:
            yaw = msg.position[msg.name.index(JOINT_YAW)]
            pitch = msg.position[msg.name.index(JOINT_PITCH)]
        except (ValueError, IndexError):
            return
        k = self.key(msg.header.stamp)
        self.truths[k] = (yaw, pitch)
        self.match(k)

    def tf_cb(self, msg):
        for tf in msg.transforms:
            self.tf_msg_count += 1
            parent = tf.header.frame_id.lstrip('/')
            child = tf.child_frame_id.lstrip('/')
            q = tf.transform.rotation
            quat = (q.x, q.y, q.z, q.w)
            k = self.key(tf.header.stamp)

            if parent == self.base_frame and child == self.yaw_frame:
                self.yaw_tf[k] = quat
                self.match(k)
            elif parent == self.yaw_frame and child == self.pitch_frame:
                self.pitch_tf[k] = quat
                self.match(k)
            elif parent == self.yaw_frame and child == self.base_frame:
                self.reversed_yaw += 1
            elif parent == self.pitch_frame and child == self.yaw_frame:
                self.reversed_pitch += 1

    def match(self, k):
        """真值和某一段 TF 都到齐时算一次误差，算完就把这一边取走"""
        if k not in self.truths:
            return
        yaw, pitch = self.truths[k]

        if k in self.yaw_tf:
            expected = quat_from_axis_angle('z', yaw)
            self.yaw_errors.append(angle_between(expected, self.yaw_tf.pop(k)))
            self.yaw_done.add(k)
        if k in self.pitch_tf:
            expected = quat_from_axis_angle('y', pitch)
            self.pitch_errors.append(angle_between(expected, self.pitch_tf.pop(k)))
            self.pitch_done.add(k)

        # 两段都比过了，这条真值不再需要
        if k in self.yaw_done and k in self.pitch_done:
            self.truths.pop(k, None)
            self.yaw_done.discard(k)
            self.pitch_done.discard(k)

    def tick(self):
        elapsed = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        if elapsed < self.duration:
            return
        self.report()
        raise SystemExit

    def stats(self, errs):
        n = len(errs)
        return n, sum(errs) / n, max(errs)

    def report(self):
        print()
        print('=' * 66)
        print('  云台关节角校验结果')
        print('=' * 66)

        if not self.yaw_errors and not self.pitch_errors:
            print('  没有配对到任何样本。按下面的顺序查：')
            print()
            if self.tf_msg_count == 0:
                print('  /tf 上一条消息都没有。locator 根本没发 TF，或者没有在运行。')
            elif self.reversed_yaw or self.reversed_pitch:
                print(f'  看到了反向的 TF：'
                      f'{self.yaw_frame}->{self.base_frame} {self.reversed_yaw} 条，'
                      f'{self.pitch_frame}->{self.yaw_frame} {self.reversed_pitch} 条。')
                print('  父子坐标系写反了：header.frame_id 应该是父，child_frame_id 是子。')
            else:
                print(f'  /tf 上收到了 {self.tf_msg_count} 条消息，但没有'
                      f' {self.base_frame}->{self.yaw_frame} 或'
                      f' {self.yaw_frame}->{self.pitch_frame} 这两段。')
                print('  检查 frame 名字是否拼写正确。')
            print()
            print(f'  收到真值 {len(self.truths)} 条。')
            if self.truths and self.tf_msg_count > 0:
                print('  如果上面两段 TF 确实在发，那就是时间戳对不上：')
                print('  TF 的时间戳必须用 IMU 消息自带的 header.stamp，不能用 now()。')
            print('=' * 66)
            self.ok = False
            return

        missing = []
        bad = []
        for label, errs in (('yaw', self.yaw_errors), ('pitch', self.pitch_errors)):
            if not errs:
                print(f'  {label:5s}：没有配对到样本')
                missing.append(label)
                continue
            n, mean, worst = self.stats(errs)
            print(f'  {label:5s}：配对 {n:5d} 个，'
                  f'平均 {math.degrees(mean) * 1e3:8.4f} mdeg，'
                  f'最大 {math.degrees(worst) * 1e3:8.4f} mdeg')
            if worst > self.tolerance:
                bad.append(label)

        print('-' * 66)
        if not missing and not bad:
            print(f'  通过：两个关节角都在容差 '
                  f'{math.degrees(self.tolerance) * 1e3:.3f} mdeg 内')
            print('  IMU 反解正确，云台 TF 的时间戳也对得上。')
            self.ok = True
            print('=' * 66)
            return

        print('  未通过。')
        print()
        if missing:
            seg = {
                'yaw': f'{self.base_frame} -> {self.yaw_frame}',
                'pitch': f'{self.yaw_frame} -> {self.pitch_frame}',
            }
            for label in missing:
                print(f'  {label} 一个样本都没配上。这一段 TF（{seg[label]}）'
                      f'可能没发出来、')
                print(f'  父子写反了、frame 名字拼错了，或者时间戳和 IMU 消息对不上。')
            if self.reversed_yaw or self.reversed_pitch:
                print(f'  另外看到了反向的 TF：'
                      f'{self.yaw_frame}->{self.base_frame} {self.reversed_yaw} 条，'
                      f'{self.pitch_frame}->{self.yaw_frame} {self.reversed_pitch} 条，'
                      f'父子确实写反了。')
            print()
        if len(bad) == 2:
            print('  两个关节角都超差。多半是反解公式：复合顺序、转置，')
            print('  或者没有先把底盘姿态除掉。')
        elif len(bad) == 1:
            print(f'  只有 {bad[0]} 超差。多半是分解出来的角填错了位置，或者绕错了轴。')
        if bad:
            print('  如果误差量级在 100 mdeg 上下而不是几万，查询 TF 用的时间可能不对，')
            print('  比如查了最新而不是消息自带的时间戳。')
        print('=' * 66)
        self.ok = False


def main():
    rclpy.init()
    node = GimbalVerifier()
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
