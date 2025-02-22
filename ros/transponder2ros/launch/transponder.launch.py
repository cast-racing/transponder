
# Launch file for the transponder

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    # Node for interfacing with the transponder
    node = Node(
        package='transponder2ros',
        executable='transponder2ros_node',
        name='transponder_node',
        output='screen',
        namespace='transponder',
        parameters=[
            # ROS network
            {'transponder_in': 'in'},    # Topic for data from the transponder
            {'transponder_out': 'out'},  # Topic for data to the transponder
            {'version_out': 'version'},  # Topic to print version information
            # Transponder network
            {'ros_ip': '127.0.0.1'},     # IP address of the computer running ros2
            {'udp_port': 15783},         # UDP port to communicate on (match transponder)
            {'max_age':  1.0},           # Max age of packets, otherwise reject [ s ]
            {'timeout': 10.0},           # Time before notifying of no packets [ s ]
        ],
    )
    ld.add_action(node)

    # Data faker node (for debugging)
    node = Node(
        package='transponder2ros',
        executable='debug_node',
        name='data_node',
        output='screen',
        namespace='transponder',
        parameters=[
            {'topic': 'out'},           # Topic to send out fake to transponder messages
        ]
    )
    ld.add_action(node)

    return ld
