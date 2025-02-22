import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    multi_robot_dir = get_package_share_directory("multi_robot_control")
    sim_dir = get_package_share_directory("nav2_minimal_tb4_sim")
    desc_dir = get_package_share_directory("nav2_minimal_tb4_description")

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    use_simulator = LaunchConfiguration("use_simulator")
    robot_name = LaunchConfiguration("robot_name")
    # robot_sdf = LaunchConfiguration("robot_sdf")
    pose = {
        "x": LaunchConfiguration("x_pose", default="-8.00"),
        "y": LaunchConfiguration("y_pose", default="-0.50"),
        "z": LaunchConfiguration("z_pose", default="0.01"),
        "R": LaunchConfiguration("roll", default="0.00"),
        "P": LaunchConfiguration("pitch", default="0.00"),
        "Y": LaunchConfiguration("yaw", default="0.00"),
    }

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        "use_simulator",
        default_value="True",
        description="Whether to start the simulator",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name", default_value="turtlebot4", description="name of the robot"
    )

    # declare_robot_sdf_cmd = DeclareLaunchArgument(
    #     "robot_sdf",
    #     default_value=os.path.join(
    #         desc_dir, "urdf", "standard", "turtlebot4.urdf.xacro"
    #     ),
    #     description="Full path to robot sdf file to spawn the robot in gazebo",
    # )

    # remappings = [
    #     ("/tf", f"{robot_name}/tf"),
    #     ("/tf_static", f"{robot_name}/tf_static"),
    #     ("/joint_states", f"{robot_name}/joint_states"),
    #     ("/scan", f"{robot_name}/scan"),
    #     ("/cmd_vel", f"{robot_name}/cmd_vel"),
    # ]
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_ros_gz",
        namespace=namespace,
        parameters=[
            {
                "config_file": os.path.join(
                    multi_robot_dir, "config", "params", "tb4_bridge.yaml"
                ),
                "expand_gz_topic_names": True,
                "use_sim_time": use_sim_time,
            }
        ],
        output="screen",
    )

    # camera_bridge_image = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",
    #     name="bridge_gz_ros_camera_image",
    #     namespace=namespace,
    #     output="screen",
    #     parameters=[
    #         {
    #             "use_sim_time": use_sim_time,
    #         }
    #     ],
    #     arguments=["/rgbd_camera/image"],
    # )
    #
    # camera_bridge_depth = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",
    #     name="bridge_gz_ros_camera_depth",
    #     namespace=namespace,
    #     output="screen",
    #     parameters=[
    #         {
    #             "use_sim_time": use_sim_time,
    #         }
    #     ],
    #     arguments=["/rgbd_camera/depth_image"],
    # )

    spawn_model = Node(
        condition=IfCondition(use_simulator),
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-entity",
            robot_name,
            "-topic",
            "/robot_description",
            # "-file",
            # Command(["xacro", " ", robot_sdf]),
            "-robot_namespace",
            namespace,
            "-x",
            pose["x"],
            "-y",
            pose["y"],
            "-z",
            pose["z"],
            "-R",
            pose["R"],
            "-P",
            pose["P"],
            "-Y",
            pose["Y"],
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", str(Path(os.path.join(desc_dir)).parent.resolve())
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    # ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(set_env_vars_resources)

    ld.add_action(bridge)
    # ld.add_action(camera_bridge_image)
    # ld.add_action(camera_bridge_depth)
    ld.add_action(spawn_model)
    return ld
