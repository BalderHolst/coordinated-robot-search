from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, log_info
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import OpaqueFunction


def spawn_robots(context, *args, **kwargs):
    n_robots_value = int(context.perform_substitution(LaunchConfiguration("n_robots")))
    print(f"Spawning {n_robots_value} robots...")
    launch = []
    node = Node(
        package="tf_topic_combiner",
        executable="tf_topic_combiner",
        name="tf_topic_combiner",
        output="screen",
        arguments=[f"{n_robots_value}"],
    )
    launch.append(node)

    # Also possible to append LaunchDescription to the launch list
    return launch


def generate_launch_description():
    declare_robots_cmd = DeclareLaunchArgument(
        "n_robots",
        default_value="1",
        description="Number of robots to spawn",
    )

    ld = LaunchDescription()
    ld.add_action(declare_robots_cmd)
    ld.add_action(
        OpaqueFunction(function=spawn_robots)
    )  # Ensures substitution happens at runtime

    return ld
