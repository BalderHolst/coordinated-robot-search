import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    n_robots = LaunchConfiguration("n_robots")
    declare_robots_cmd = DeclareLaunchArgument(
        "n_robots",
        default_value="1",
        description="Number of robots to spawn",
    )

    # Make n_robots to int
    n_robots_value = int(...)
    print(n_robots_value)
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_robots_cmd)

    return ld
