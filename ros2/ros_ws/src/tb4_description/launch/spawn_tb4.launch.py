import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    robot_name = LaunchConfiguration("robot_name")

    # Paths
    desc_dir = get_package_share_directory("tb4_description")

    pose = {
        "x": LaunchConfiguration("x_pose", default="-8.00"),
        "y": LaunchConfiguration("y_pose", default="-0.50"),
        "z": LaunchConfiguration("z_pose", default="0.01"),
        "R": LaunchConfiguration("roll", default="0.00"),
        "P": LaunchConfiguration("pitch", default="0.00"),
        "Y": LaunchConfiguration("yaw", default="0.00"),
    }

    robot_sdf = os.path.join(desc_dir, "urdf", "standard", "turtlebot4.urdf.xacro")
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "frame_prefix": namespace,
                "robot_description": Command(
                    ["xacro", " ", robot_sdf, " namespace:=", namespace]
                ),
            }
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    robot_bridge_namespaced = ReplaceString(
        source_file=os.path.join(desc_dir, "params", "bridge_robot_tmp.yaml"),
        replacements={"<gz_namespace>": ("/", namespace)},
    )
    bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        namespace=namespace,
        parameters=[
            {
                "config_file": robot_bridge_namespaced,
                "use_sim_time": use_sim_time,
            }
        ],
        output="screen",
    )

    spawn_model_cmd = Node(
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
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace", default_value="", description="Top-level namespace"
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="turtlebot4",
                description="name of the robot",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                str(Path(os.path.join(desc_dir)).parent.resolve()),
            ),
            start_robot_state_publisher_cmd,
            bridge_cmd,
            spawn_model_cmd,
        ]
    )

    return ld
