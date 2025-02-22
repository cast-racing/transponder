
# Launch file for the transponder

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    ld = LaunchDescription()

    # Robot config
    config_dir = os.path.join(
        get_package_share_directory('robot_description'),
        'config',
        'av24.yaml'
    )
    robot_arg = DeclareLaunchArgument('robot', default_value=config_dir)
    ld.add_action(robot_arg)

    # Node for interfacing with the transponder
    node = Node(
        package='transponder2ros',
        executable='transponder2ros_node',
        name='transponder_node',
        output='screen',
        namespace='transponder',
        # arguments=[('__log_level:=debug')]),
        parameters=[
            {'transponder_in': 'in'},
            {'transponder_out': 'out'},
            {'version_out': 'version'},
            LaunchConfiguration('robot'),
        ]
    )
    ld.add_action(node)

    # Data faker node
    node = Node(
        package='transponder2ros',
        executable='debug_node',
        name='data_node',
        output='screen',
        namespace='transponder',
        # arguments=[('__log_level:=debug')]),
        parameters=[
            {'topic': 'out'},
            LaunchConfiguration('robot'),
        ]
    )
    ld.add_action(node)

    return ld
