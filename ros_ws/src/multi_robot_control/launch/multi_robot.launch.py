import os
from dataclasses import dataclass
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    sim_dir = get_package_share_directory("multi_robot_control")
    desc_dir = get_package_share_directory("tb4_description")

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Launch configuration variables specific to simulation
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")

    # The Gazebo command line doesn't take SDF strings for worlds, so the output of xacro needs to be saved into
    # a temporary file and passed to Gazebo.
    world_sdf = tempfile.mktemp(prefix="tb4_", suffix=".sdf")
    world_sdf_xacro = ExecuteProcess(
        cmd=["xacro", "-o", world_sdf, ["headless:=", headless], world]
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": ["-r -s ", world_sdf]}.items(),
    )

    remove_temp_sdf_file = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))]
        )
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        condition=UnlessCondition(headless),
        launch_arguments={"gz_args": ["-v4 -g "]}.items(),
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(sim_dir, "config", "rviz", "map_overlay.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Create the launch description and populate
    ld = LaunchDescription(
        [
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", os.path.join(sim_dir, "config", "worlds")
            ),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                str(Path(os.path.join(desc_dir)).parent.resolve()),
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="False",
                description="Whether to execute gzclient)",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="True",
                description="Use rviz",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(sim_dir, "config", "worlds", "depot.sdf"),
                description="Full path to xacro world model file to load",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(sim_dir, "config", "maps", "depot.yaml"),
                description="Full path to map yaml file to load",
            ),
            DeclareLaunchArgument(
                "robots",
                default_value="1",
                description="How many robots to spawn (a number) or a list of robot poses (ex \"0,0,0:1,0,0\")",
            ),
            DeclareLaunchArgument(
                "behavior",
                default_value="dumb:circle",
                description="What behavior to run on the robots",
            ),
            DeclareLaunchArgument(
                "use_control",
                default_value="True",
                description="Whether to run the control node",
            ),
            world_sdf_xacro,
            remove_temp_sdf_file,
            gazebo_server,
            gazebo_client,
            rviz,
            clock_bridge,
            OpaqueFunction(
                function=spawn_robots
            ),  # Ensures substitution happens at runtime
        ]
    )
    return ld

@dataclass
class RobotPose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.1
    angle: float = 0.0

    def to_amcl_pose(self):
        return {
            "x": str(self.x),
            "y": str(self.y),
            "z": str(self.z),
            "R": "0.00",
            "P": "0.00",
            "Y": str(self.angle),
        }

def get_robot_poses(context) -> list[RobotPose]:
    robots_arg = context.perform_substitution(LaunchConfiguration("robots"))

    poses = []

    try:
        n = int(robots_arg)
        poses = [RobotPose(x=i - n/2) for i in range(n)]

    except ValueError:
        for str_pose in robots_arg.split(":"):
            try:
                x, y, angle = str_pose.split(",")
                poses.append(RobotPose(x=float(x), y=float(y), angle=float(angle)))
            except ValueError:
                print(f"Invalid robot pose: {str_pose}")
                exit(1)

        print("Something went wrong with the robots value")

    return poses


def spawn_robots(context):
    desc_dir = get_package_share_directory("tb4_description")
    sim_dir = get_package_share_directory("multi_robot_control")

    robot_poses = get_robot_poses(context)

    use_sim_time = LaunchConfiguration("use_sim_time")
    behavior = LaunchConfiguration("behavior")
    print(f"Spawning {len(robot_poses)} robots...")

    robot_launch = []

    tf_combiner = Node(
        package="multi_robot_control",
        executable="tf_topic_combiner",
        name="tf_topic_combiner",
        output="screen",
        arguments=[f"{len(robot_poses)}"],
    )
    robot_launch.append(tf_combiner)

    params_file = os.path.join(sim_dir, "config", "nav2", "amcl.yaml")

    for i, pose in enumerate(robot_poses):

        namespace = f"robot_{i}"
        print(f"Launching robot {pose} with namespace '{namespace}'")
        gz_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_dir, "launch", "spawn_tb4.launch.py")
            ),
            launch_arguments = {
                "namespace": namespace + "/",
                "robot_name": namespace,
                "use_sim_time": use_sim_time,
                "x_pose": str(pose.x),
                "y_pose": str(pose.y),
                "z_pose": str(pose.z),
                "roll": "0.0",
                "pitch": "0.0",
                "yaw": str(pose.angle),
            }.items(),
        )

        robot_launch.append(gz_robot)

        print("Starting AMCL for " + namespace)

        # Only used to add namespace to topics
        configured_params = ParameterFile(
            RewrittenYaml(
                source_file=params_file,
                root_key=namespace,
                param_rewrites={
                    "base_frame_id": namespace + "/base_link",
                    "odom_frame_id": namespace + "/odom",
                    "x": str(pose.x),
                    "y": str(pose.y),
                    "yaw": str(pose.angle),
                },
                convert_types=True,
            ),
            allow_substs=True,
        )

        nav2_amcl = Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            namespace=namespace,
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", "info"],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        )
        lifecycle_node = Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            namespace=namespace,
            output="screen",
            arguments=["--ros-args", "--log-level", "info"],
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": True},
                {"node_names": ["amcl"]},
            ],
        )
        robot_launch.append(lifecycle_node)
        robot_launch.append(nav2_amcl)

        print("Starting behavior for " + namespace)
        robot_launch.append(
            Node(
                condition=IfCondition(LaunchConfiguration("use_control")),
                package="multi_robot_control",
                executable="ros_agent",
                namespace=namespace,
                name="ros_agent",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, {"behavior": behavior}],
            )
        )

    map_yaml = LaunchConfiguration("map")
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[{"yaml_filename": map_yaml}],
        arguments=["--ros-args", "--log-level", "warn"],
    )
    robot_launch.append(map_server)
    lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )
    robot_launch.append(lifecycle_node)

    return robot_launch
