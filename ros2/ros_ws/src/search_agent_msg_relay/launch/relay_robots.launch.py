from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    namespace_arg = DeclareLaunchArgument(
        'global_topic',
        default_value='search_channel',
        description='The namespace for the robot'
    )

    n_robots_arg = DeclareLaunchArgument(
        'n_robots',
        default_value='1',
        description='The number of robots'
    )


    return LaunchDescription([
        namespace_arg,
        n_robots_arg,
        OpaqueFunction(function=spawn_nodes),
    ])


def spawn_nodes(context, *args, **kwargs):
    n_robots = int(context.perform_substitution(LaunchConfiguration("n_robots")))
    global_topic = str(context.perform_substitution(LaunchConfiguration('global_topic')))

    nodes = []
    for i in range(int(n_robots)):
        robot_namespace = f"robot_{i}"
        nodes.append(Node(
            package='search_agent_msg_relay',
            executable='search_agent_msg_relay',
            namespace=robot_namespace,  # Set the dynamic namespace
            parameters=[{
                'robot_namespace': robot_namespace,
                'global_topic': global_topic,
            }],
        ))

    return nodes
