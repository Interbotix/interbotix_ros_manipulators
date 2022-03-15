from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_name",
            default_value=TextSubstitution(text=""),
            description=(
                "name of the robot (typically equal to robot_model, but could be "
                "anything)'"
            ),
        ),
        DeclareLaunchArgument(
            "rvizconfig",
            default_value=TextSubstitution(text=""),
            description="file path to the config file RViz should load",
        ),
    ]

    xsarm_descriptions_prefix = get_package_share_directory(
        "interbotix_xsarm_descriptions"
    )

    robot_name = LaunchConfiguration("robot_name")
    rvizconfig = LaunchConfiguration("rvizconfig")

    rvizconfig = PathJoinSubstitution(
        [xsarm_descriptions_prefix, "rviz", "xsarm_description.rviz"]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=robot_name,
        name="rviz2",
        output="log",
        arguments=["-d", rvizconfig],
    )

    nodes_to_start = [
        rviz2_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
