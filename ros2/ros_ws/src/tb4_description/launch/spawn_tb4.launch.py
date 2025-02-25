import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    sim_dir = get_package_share_directory("tb4_description")

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    use_simulator = LaunchConfiguration("use_simulator")
    robot_name = LaunchConfiguration("robot_name")
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

    robot_bridge = os.path.join(sim_dir, "launch", "bridge_robot_tmp.yaml")
    camera_bridge_image = os.path.join(
        sim_dir, "launch", "bridge_camera_image_tmp.yaml"
    )
    camera_bridge_depth = os.path.join(
        sim_dir, "launch", "bridge_camera_depth_tmp.yaml"
    )

    robot_bridge_namespaced = ReplaceString(
        source_file=robot_bridge,
        replacements={"<gz_namespace>": ("/", namespace)},
    )
    camera_bridge_image_namespaced = ReplaceString(
        source_file=camera_bridge_image,
        replacements={"<gz_namespace>": ("/", namespace)},
    )
    camera_bridge_depth_namespaced = ReplaceString(
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

    camera_bridge_image = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="bridge_gz_ros_camera_image",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "config_file": camera_bridge_image_namespaced,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    camera_bridge_depth = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="bridge_gz_ros_camera_depth",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "config_file": camera_bridge_depth_namespaced,
                "use_sim_time": use_sim_time,
            }
        ],
        arguments=["rgbd_camera/depth_image"],
    )

    spawn_model = Node(
        condition=IfCondition(use_simulator),
        package="ros_gz_sim",
        executable="create",
        namespace=namespace,
        output="screen",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            "robot_description",
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

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(bridge)
    ld.add_action(camera_bridge_image)
    ld.add_action(camera_bridge_depth)
    ld.add_action(spawn_model)
    return ld
