import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('autoaim_detector')
    params_file = os.path.join(share, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),

        Node(
            package='autoaim_detector',
            executable='autoaim_detector_node',
            name='autoaim_detector',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[
                {'port': ParameterValue(
                    LaunchConfiguration('foxglove_port'),
                    value_type=int)}
            ],
            condition=IfCondition(LaunchConfiguration('foxglove')),
        ),
    ])
