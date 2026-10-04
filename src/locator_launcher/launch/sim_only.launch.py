import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """只启动仿真侧，不启动 locator。

    这个文件是给你当参照的，不是本次要交的东西。你要写的是
    locator_launcher.launch.py，要求见 launch/README.md。

    跑起来之后，Foxglove 里能看到：
      - TF 树只有 world -> base_link 这一段（云台那几段要由 locator 发布）
      - 车体、轮子、云台、相机、视场锥这些 marker
      - 世界系下的卡西尼卵形线真值曲线，以及沿着它飞的目标球

    用法：
        ros2 launch locator_launcher sim_only.launch.py

    四个仿真节点放在同一个容器（也就是同一个进程）里，这是硬性要求：它们靠一个
    进程内共享的静态 t0 对齐仿真起始时刻。拆成多个进程时不会报任何错，但各节点
    算出的真值属于不同的世界。
    """
    sim_params = os.path.join(
        get_package_share_directory('locator_sim'), 'config', 'sim_params.yaml'
    )

    sim_nodes = [
        ('locator_sim::VirtualSerialNode', 'virtual_serial'),
        ('locator_sim::VirtualImuNode', 'virtual_imu'),
        ('locator_sim::VirtualDetectorNode', 'virtual_detector'),
        ('locator_sim::SimVisualizerNode', 'sim_visualizer'),
    ]

    return LaunchDescription([
        ComposableNodeContainer(
            name='sim_container',
            namespace='',
            package='rclcpp_components',
            # 多线程执行器。单线程容器下，任何一个回调阻塞都会让其他节点的
            # 定时器发不出数据。
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='locator_sim',
                    plugin=plugin,
                    name=name,
                    parameters=[sim_params],
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
                for plugin, name in sim_nodes
            ],
            output='both',
            emulate_tty=True,
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='log',
            parameters=[{'port': 8765}],
        ),
    ])
