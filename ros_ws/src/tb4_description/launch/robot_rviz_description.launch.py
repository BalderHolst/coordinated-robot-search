from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="use_sim_time",
    ),
    DeclareLaunchArgument(
        "robot_name", default_value="turtlebot4", description="Robot name"
    ),
    DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
    DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        choices=["true", "false"],
        description="Whether to apply a namespace",
    ),
]


def generate_launch_description():
    pkg_turtlebot4_description = get_package_share_directory("tb4_description")
    xacro_file = PathJoinSubstitution(
        [pkg_turtlebot4_description, "urdf", "standard", "turtlebot4.urdf.xacro"]
    )
    rviz_config_file = PathJoinSubstitution(
        [pkg_turtlebot4_description, "rviz", "config.rviz"]
    )
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        namespace=namespace,
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {
                "robot_description": Command(
                    ["xacro", " ", xacro_file, " ", "namespace:=", namespace]
                )
            },
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        namespace=namespace,
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    namespaced_rviz_config_file = ReplaceString(
        condition=IfCondition(use_namespace),
        source_file=rviz_config_file,
        replacements={"<robot_namespace>": ("/", namespace)},
    )

    namespaced_rviz2 = Node(
        condition=IfCondition(use_namespace),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", namespaced_rviz_config_file],
        namespace=namespace,
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/map", "map"),
            ("/goal_pose", "goal_pose"),
            ("/clicked_point", "clicked_point"),
            ("/initialpose", "initialpose"),
        ],
    )

    normal_rviz_config_file = ReplaceString(
        condition=UnlessCondition(use_namespace),
        source_file=rviz_config_file,
        replacements={"<robot_namespace>": ("")},
    )

    normal_rviz2 = Node(
        condition=UnlessCondition(use_namespace),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", normal_rviz_config_file],
        namespace=namespace,
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/map", "map"),
            ("/goal_pose", "goal_pose"),
            ("/clicked_point", "clicked_point"),
            ("/initialpose", "initialpose"),
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(namespaced_rviz2)
    ld.add_action(normal_rviz2)
    return ld
