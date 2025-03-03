import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    robot_name = LaunchConfiguration("robot_name")

    # Paths
    desc_dir = get_package_share_directory("tb4_description")
    robot_bridge = os.path.join(desc_dir, "params", "bridge_robot_tmp.yaml")

    pose = {
        "x": LaunchConfiguration("x_pose", default="-8.00"),
        "y": LaunchConfiguration("y_pose", default="-0.50"),
        "z": LaunchConfiguration("z_pose", default="0.01"),
        "R": LaunchConfiguration("roll", default="0.00"),
        "P": LaunchConfiguration("pitch", default="0.00"),
        "Y": LaunchConfiguration("yaw", default="0.00"),
    }

    robot_bridge_namespaced = ReplaceString(
        source_file=robot_bridge,
        replacements={"<gz_namespace>": ("/", namespace)},
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=robot_name,
        namespace=namespace,
        parameters=[
            {
                "config_file": robot_bridge_namespaced,
                "use_sim_time": use_sim_time,
            }
        ],
        output="screen",
    )

    spawn_model = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=namespace,
        output="screen",
        arguments={
            "-name=": robot_name,
            "-topic=": "robot_description",
            "-x=": pose["x"],
            "-y=": pose["y"],
            "-z=": pose["z"],
            "-R=": pose["R"],
            "-P=": pose["P"],
            "-Y=": pose["Y"],
        }.items(),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )
    DeclareLaunchArgument(
        "robot_name", default_value="turtlebot4", description="name of the robot"
    )
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(desc_dir))
    ld.add_action(bridge)
    ld.add_action(spawn_model)
    return ld
