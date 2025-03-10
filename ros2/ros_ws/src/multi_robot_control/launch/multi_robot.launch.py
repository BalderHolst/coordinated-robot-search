import os
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
from launch.conditions import UnlessCondition
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

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        arguments=["/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock"],
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
                "world",
                default_value=os.path.join(sim_dir, "config", "worlds", "depot.sdf"),
                description="Full path to world model file to load",
            ),
            DeclareLaunchArgument(
                "n_robots",
                default_value="1",
                description="How many robots to spawn",
            ),
            world_sdf_xacro,
            remove_temp_sdf_file,
            gazebo_server,
            gazebo_client,
            clock_bridge,
            OpaqueFunction(
                function=spawn_robots
            ),  # Ensures substitution happens at runtime
        ]
    )
    return ld


def spawn_robots(context, *args, **kwargs):
    desc_dir = get_package_share_directory("tb4_description")
    sim_dir = get_package_share_directory("multi_robot_control")

    use_sim_time = LaunchConfiguration("use_sim_time")
    n_robots_value = int(context.perform_substitution(LaunchConfiguration("n_robots")))
    print(f"Spawning {n_robots_value} robots...")

    robot_launch = []
    tf_combiner = Node(
        package="multi_robot_control",
        executable="tf_topic_combiner",
        name="tf_topic_combiner",
        output="screen",
        arguments=[f"{n_robots_value}"],
    )
    robot_launch.append(tf_combiner)

    params_file = os.path.join(sim_dir, "config", "nav2", "amcl.yaml")
    for i in range(n_robots_value):
        pose = {
            "x": str(-8.00 + float(i)),
            "y": "0.00",
            "z": "0.01",
            "R": "0.00",
            "P": "0.00",
            "Y": "0.00",
        }
        namespace = f"robot_{i}"
        print("Launching robot " + namespace)
        gz_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_dir, "launch", "spawn_tb4.launch.py")
            ),
            launch_arguments={
                "namespace": namespace + "/",
                "robot_name": namespace,
                "use_sim_time": use_sim_time,
                "x_pose": pose["x"],
                "y_pose": pose["y"],
                "z_pose": pose["z"],
                "roll": pose["R"],
                "pitch": pose["P"],
                "yaw": pose["Y"],
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
                    "x": str(float(i)),
                    # "y": str(float(i)),   # Not relevant ATM
                    # "yam": str(float(i)), # Not relevant ATM
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

    map_yaml = os.path.join(sim_dir, "config", "maps", "depot.yaml")
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
