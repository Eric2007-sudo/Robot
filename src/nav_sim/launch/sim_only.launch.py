# 只启动仿真（地图 + 底盘，不含导航节点）的示例 launch。
#
# 这个文件只用于单独检查地图、TF、仿真和 Foxglove；完整系统使用 nav_launcher 里的
# nav.launch.py。
#
# 跑起来之后，Foxglove（连 ws://localhost:8765）里能看到：
#   - /map_server/global_cost_map   一张真实比赛场地的代价图（障碍 + 连续梯度膨胀）
#   - TF 树                         map -> odom -> base_link，base_link（车）停在起点
#   - /nav_sim/odom、/nav_sim/ground_truth   里程计与当前真值（都静止）
#   - /nav_sim/ground_truth_path            实际行驶轨迹（静止时只有起点）
#   - /nav_sim/map_velocity                 地图坐标系下的实际 x/y 速度
#
# 因为这里不启动导航节点，所以看不到 global_path；车会停在起点等待指令。
#
# 用法：
#     ros2 launch nav_sim sim_only.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sim_params = os.path.join(
        get_package_share_directory('nav_sim'), 'config', 'sim_params.yaml'
    )
    map_params = os.path.join(
        get_package_share_directory('map_server'), 'config', 'map_server_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='map_server',
            executable='map_server_node',
            name='map_server',
            parameters=[map_params],
            output='screen',
        ),
        Node(
            package='nav_sim',
            executable='robot_sim',
            name='robot_sim',
            parameters=[sim_params],
            output='screen',
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='log',
            parameters=[{'port': 8765}],
        ),
    ])
