from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='search_agent_msg_relay',
            executable='relay_node',
            parameters=[{'namespace': 'robot1'}],
        )
    ])
