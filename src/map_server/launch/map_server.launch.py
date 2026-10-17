# 只启动地图服务器的示例 launch。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('map_server'), 'config', 'map_server_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='map_server',
            executable='map_server_node',
            name='map_server',
            parameters=[params],
            output='screen',
        ),
    ])
