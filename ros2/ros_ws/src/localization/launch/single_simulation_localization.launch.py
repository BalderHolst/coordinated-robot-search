# This file is the predecessor of multi_robot.launch.py
#
import os
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
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    sim_dir = get_package_share_directory("nav2_minimal_tb4_sim")  # For world file
    desc_dir = get_package_share_directory("tb4_description")  # For description files
    localization_dir = get_package_share_directory(
        "localization"
    )  # Only for the amcl.yaml file and map

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Launch configuration variables specific to simulation
    use_simulator = LaunchConfiguration("use_simulator")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    pose = {
        "x": LaunchConfiguration("x_pose", default="-8.00"),
        "y": LaunchConfiguration("y_pose", default="0.00"),
        "z": LaunchConfiguration("z_pose", default="0.01"),
        "R": LaunchConfiguration("roll", default="0.00"),
        "P": LaunchConfiguration("pitch", default="0.00"),
        "Y": LaunchConfiguration("yaw", default="0.00"),
    }
    map_server_yaml = LaunchConfiguration("map_server_yaml")

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
        condition=IfCondition(use_simulator),
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
        condition=IfCondition(PythonExpression([use_simulator, " and not ", headless])),
        launch_arguments={"gz_args": ["-v4 -g "]}.items(),
    )

    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_dir, "launch", "spawn_tb4.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
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
    params_file = os.path.join(localization_dir, "params", "amcl.yaml")
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params, {"yaml_filename": map_server_yaml}],
        # parameters=[{"yaml_filename": map_server_yaml}],
        arguments=["--ros-args", "--log-level", "info"],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    # nav2_amcl = Node(
    #     package="nav2_amcl",
    #     executable="amcl",
    #     name="amcl",
    #     output="screen",
    #     respawn=True,
    #     respawn_delay=2.0,
    #     parameters=[configured_params],
    #     arguments=["--ros-args", "--log-level", "info"],
    #     remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    # )
    # lifecycle_nodes = ["map_server", "amcl"]
    lifecycle_nodes = ["map_server"]
    lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": lifecycle_nodes},
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription(
        [
            # Declare the launch options
            DeclareLaunchArgument(
                "namespace", default_value="", description="Top-level namespace"
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "use_simulator",
                default_value="True",
                description="Whether to start the simulator",
            ),
            DeclareLaunchArgument(
                "use_robot_state_pub",
                default_value="True",
                description="Whether to start the robot state publisher",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="False",
                description="Whether to execute gzclient)",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(sim_dir, "worlds", "depot.sdf"),
                description="Full path to world model file to load",
            ),
            DeclareLaunchArgument(
                "map_server_yaml",
                default_value=os.path.join(localization_dir, "maps", "depot.yaml"),
                description="Full path to map yaml file to load",
            ),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", os.path.join(sim_dir, "worlds")
            ),
            # world_sdf_xacro,
            # remove_temp_sdf_file,
            # gz_robot,
            # gazebo_server,
            # gazebo_client,
            # clock_bridge,
            map_server,
            lifecycle_node,
            # nav2_amcl,
        ]
    )
    return ld
