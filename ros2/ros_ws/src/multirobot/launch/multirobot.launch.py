import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from nav2_common.launch import ParseMultiRobotPose


def generate_launch_description():
    ########### General imports ###########
    bringup_dir = get_package_share_directory("nav2_bringup")
    bringup_launch_dir = os.path.join(bringup_dir, "launch")
    sim_dir = get_package_share_directory("nav2_minimal_tb4_sim")
    desc_dir = get_package_share_directory("nav2_minimal_tb4_description")
    multi_robot = get_package_share_directory("multi_robot_control")

    ########### Variables ###########
    world = LaunchConfiguration("world")
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(multi_robot, "config", "worlds", "depot.sdf"),
        description="Full path to world model file to load",
    )

    # map_yaml_file = LaunchConfiguration("map")
    # declare_map_yaml_cmd = DeclareLaunchArgument(
    #     "map",
    #     default_value=os.path.join(bringup_dir, "maps", "tb3_sandbox.yaml"),
    #     description="Full path to map file to load",
    # )

    headless = LaunchConfiguration("headless")
    declare_headless_cmd = DeclareLaunchArgument(
        "headless",
        default_value="True",
        description="Whether to start Gazebo headless",
    )
    # use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    # declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    #     "use_robot_state_pub",
    #     default_value="True",
    #     description="Whether to start the robot state publisher",
    # )

    # use_rviz = LaunchConfiguration("use_rviz")
    # declare_use_rviz_cmd = DeclareLaunchArgument(
    #     "use_rviz", default_value="True", description="Whether to start RVIZ"
    # )

    # rviz_config_file = LaunchConfiguration("rviz_config")
    # declare_rviz_config_file_cmd = DeclareLaunchArgument(
    #     "rviz_config",
    #     default_value=os.path.join(bringup_dir, "rviz", "nav2_namespaced_view.rviz"),
    #     description="Full path to the RVIZ config file to use.",
    # )

    # params_file = LaunchConfiguration("params_file")
    # declare_params_file_cmd = DeclareLaunchArgument(
    #     "params_file",
    #     default_value=os.path.join(
    #         bringup_dir, "params", "nav2_multirobot_params_all.yaml"
    #     ),
    #     description="Full path to the ROS2 parameters file to use for all launched nodes",
    # )

    # autostart = LaunchConfiguration("autostart") # Nav2
    # declare_autostart_cmd = DeclareLaunchArgument(
    #     "autostart",
    #     default_value="false",
    #     description="Automatically startup the stacks",
    # )

    log_settings = LaunchConfiguration("log_settings", default="true")
    declare_log_settings_cmd = DeclareLaunchArgument(
        "log_settings",
        default_value="true",
        description="Whether to log settings",
    )

    ########### Robots ###########
    # TODO: Change to params file or LaunchConfiguration
    n_robots = 2
    # n_robots = LaunchConfiguration("n_robots")
    # declare_robots_cmd = DeclareLaunchArgument(
    #     "n_robots",
    #     default_value="1",
    #     description="Number of robots to spawn",
    # )

    ########### Gazebo Simulation ###########
    world_sdf = tempfile.mktemp(prefix="nav2_", suffix=".sdf")
    # Load world file with Gazebo plugins
    world_sdf_xacro = ExecuteProcess(
        cmd=["xacro", "-o", world_sdf, ["headless:=", headless], world]
    )
    # Start Gazebo server with world file
    gazebo_server_cmd = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world_sdf],
        output="screen",
    )
    # Optionally start Gazebo client
    gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        condition=UnlessCondition(headless),
        launch_arguments={"gz_args": ["-v4 -g "]}.items(),
    )
    # Clean up
    remove_temp_sdf_file = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))]
        )
    )

    ########### Robot spawn + config ###########
    robots_list = {}
    for i in range(n_robots):
        robots_list[f"robot_{i}"] = {
            "x": i * 1.0,
            "y": 0.0,
            "z": 1.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }

    # Define commands for launching the navigation instances
    bringup_cmd_group = []
    for robot_name in robots_list:
        init_pose = robots_list[robot_name]
        group = GroupAction(
            [
                LogInfo(
                    msg=[
                        "Launching namespace=",
                        robot_name,
                        " init_pose=",
                        str(init_pose),
                    ]
                ),
                # IncludeLaunchDescription(
                #     PythonLaunchDescriptionSource(
                #         os.path.join(launch_dir, "rviz_launch.py")
                #     ),
                #     condition=IfCondition(use_rviz),
                #     launch_arguments={
                #         "namespace": TextSubstitution(text=robot_name),
                #         "use_namespace": "True",
                #         "rviz_config": rviz_config_file,
                #     }.items(),
                # ),
                # IncludeLaunchDescription(
                #     PythonLaunchDescriptionSource(
                #         os.path.join(bringup_dir, "launch", "tb3_simulation_launch.py")
                #     ),
                #     launch_arguments={
                #         "namespace": robot_name,
                #         "use_namespace": "True",
                #         "map": map_yaml_file,
                #         "use_sim_time": "True",
                #         "params_file": params_file,
                #         "autostart": autostart,
                #         "use_rviz": "False",
                #         "use_simulator": "False",
                #         "headless": "False",
                #         "use_robot_state_pub": use_robot_state_pub,
                #         "x_pose": TextSubstitution(text=str(init_pose["x"])),
                #         "y_pose": TextSubstitution(text=str(init_pose["y"])),
                #         "z_pose": TextSubstitution(text=str(init_pose["z"])),
                #         "roll": TextSubstitution(text=str(init_pose["roll"])),
                #         "pitch": TextSubstitution(text=str(init_pose["pitch"])),
                #         "yaw": TextSubstitution(text=str(init_pose["yaw"])),
                #         "robot_name": TextSubstitution(text=robot_name),
                #     }.items(),
                # ),
            ]
        )
        bringup_cmd_group.append(group)

    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(sim_dir, "worlds")
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(set_env_vars_resources)

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    # ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_headless_cmd)
    # ld.add_action(declare_use_robot_state_pub_cmd)
    # ld.add_action(declare_use_rviz_cmd)
    # ld.add_action(declare_rviz_config_file_cmd)
    # ld.add_action(declare_params_file_cmd)
    # ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_settings_cmd)
    # ld.add_action(declare_robots_cmd)

    # Add the actions to start gazebo with world
    ld.add_action(world_sdf_xacro)
    ld.add_action(gazebo_server_cmd)
    ld.add_action(gazebo_client_cmd)
    ld.add_action(remove_temp_sdf_file)

    # Log settings
    # ld.add_action(
    #     LogInfo(condition=IfCondition(log_settings), msg=["map yaml: ", map_yaml_file])
    # )
    # ld.add_action(
    #     LogInfo(condition=IfCondition(log_settings), msg=["params yaml: ", params_file])
    # )
    # ld.add_action(
    #     LogInfo(
    #         condition=IfCondition(log_settings),
    #         msg=["rviz config file: ", rviz_config_file],
    #     )
    # )
    # ld.add_action(
    #     LogInfo(
    #         condition=IfCondition(log_settings),
    #         msg=["using robot state pub: ", use_robot_state_pub],
    #     )
    # )
    # ld.add_action(
    #     LogInfo(condition=IfCondition(log_settings), msg=["autostart: ", autostart])
    # )

    # Spawn robots
    for cmd in bringup_cmd_group:
        ld.add_action(cmd)

    return ld
