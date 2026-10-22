# 一键启动整套导航系统：
#   - map_server / robot_sim / nav_executor 三个组件加载进 component_container_mt
#   - foxglove_bridge 独立启动，默认端口 8765
#
# 用法：
#   ros2 launch nav_launcher nav.launch.py
#   ros2 launch nav_launcher nav.launch.py use_foxglove:=false
#   ros2 launch nav_launcher nav.launch.py foxglove_port:=9000

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    map_params = os.path.join(
        get_package_share_directory('map_server'),
        'config', 'map_server_params.yaml')
    sim_params = os.path.join(
        get_package_share_directory('nav_sim'),
        'config', 'sim_params.yaml')
    nav_params = os.path.join(
        get_package_share_directory('nav_executor'),
        'config', 'nav_params.yaml')

    use_foxglove = LaunchConfiguration('use_foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')

    container = ComposableNodeContainer(
        name='nav_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='map_server',
                plugin='map_server::MapServerNode',
                name='map_server',
                parameters=[map_params],
            ),
            ComposableNode(
                package='nav_sim',
                plugin='nav_sim::RobotSimNode',
                name='robot_sim',
                parameters=[sim_params],
            ),
            ComposableNode(
                package='nav_executor',
                plugin='nav_executor::NavExecutorNode',
                name='nav_executor',
                parameters=[nav_params],
            ),
        ],
        output='screen',
    )

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': ParameterValue(foxglove_port, value_type=int),
        }],
        condition=IfCondition(use_foxglove),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_foxglove',
            default_value='true',
            description='是否启动 foxglove_bridge'),
        DeclareLaunchArgument(
            'foxglove_port',
            default_value='8765',
            description='Foxglove WebSocket 端口'),
        container,
        foxglove,
    ])
