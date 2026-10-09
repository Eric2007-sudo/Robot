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
    sim_params = os.path.join(
        get_package_share_directory('locator_sim'), 'config', 'sim_params.yaml'
    )
    locator_params = os.path.join(
        get_package_share_directory('locator'), 'config', 'locator_params.yaml'
    )
    override = LaunchConfiguration('override_params')
    use_foxglove = LaunchConfiguration('use_foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')

    sim_nodes = [
        ('locator_sim::VirtualSerialNode', 'virtual_serial'),
        ('locator_sim::VirtualImuNode', 'virtual_imu'),
        ('locator_sim::VirtualDetectorNode', 'virtual_detector'),
        ('locator_sim::SimVisualizerNode', 'sim_visualizer'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_foxglove',
            default_value='true',
            description='whether to start foxglove_bridge',
        ),
        DeclareLaunchArgument(
            'foxglove_port',
            default_value='8765',
            description='WebSocket port for foxglove_bridge',
        ),
        DeclareLaunchArgument(
            'override_params',
            default_value=os.path.join(
                get_package_share_directory('locator_launcher'), 'config', 'empty.yaml'
            ),
            description='extra YAML overriding any parameter',
        ),
        ComposableNodeContainer(
            name='locator_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='locator_sim',
                    plugin=plugin,
                    name=name,
                    parameters=[sim_params, override],
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
                for plugin, name in sim_nodes
            ] + [
                ComposableNode(
                    package='locator',
                    plugin='locator::LocatorNode',
                    name='locator',
                    parameters=[locator_params, override],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='both',
            emulate_tty=True,
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='log',
            parameters=[{'port': ParameterValue(foxglove_port, value_type=int)}],
            condition=IfCondition(use_foxglove),
        ),
    ])
