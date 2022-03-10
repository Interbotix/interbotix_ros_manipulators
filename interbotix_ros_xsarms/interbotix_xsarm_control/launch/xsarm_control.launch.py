import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    robot_model = LaunchConfiguration("robot_model")
    robot_name = LaunchConfiguration("robot_name")
    base_link_frame = LaunchConfiguration("base_link_frame")
    show_ar_tag = LaunchConfiguration("show_ar_tag")
    show_gripper_bar = LaunchConfiguration("show_gripper_bar")
    show_gripper_fingers = LaunchConfiguration("show_gripper_fingers")
    use_world_frame = LaunchConfiguration("use_world_frame")
    external_urdf_loc = LaunchConfiguration("external_urdf_loc")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim = LaunchConfiguration("use_sim")
    load_gazebo_configs = LaunchConfiguration("load_gazebo_configs")
    model = LaunchConfiguration("model")

    motor_configs = PathJoinSubstitution([get_package_share_directory("interbotix_xsarm_control"),"config", robot_model.perform(context) + ".yaml"])
    
    mode_configs = PathJoinSubstitution([get_package_share_directory("interbotix_xsarm_control"),"config", "modes.yaml"])

    load_configs = LaunchConfiguration("load_configs", default="false")

    description_prefix = get_package_share_directory("interbotix_xsarm_descriptions")

    urdf_path = PathJoinSubstitution([get_package_share_directory("interbotix_xsarm_descriptions"), "urdf", robot_model])

    model = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_path,
            ".urdf.xacro" " ",
            "robot_name:=",
            robot_name,
            " ",
            "base_link_frame:=",
            base_link_frame,
            " ",
            "show_ar_tag:=",
            show_ar_tag,
            " ",
            "show_gripper_bar:=",
            show_gripper_bar,
            " ",
            "show_gripper_fingers:=",
            show_gripper_fingers,
            " ",
            "use_world_frame:=",
            use_world_frame,
            " ",
            "external_urdf_loc:=",
            external_urdf_loc,
            " ",
            "load_gazebo_configs:=",
            load_gazebo_configs,
        ]
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [description_prefix, "launch", "xsarm_description.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "robot_model": robot_model,
            "robot_name": robot_name,
            "base_link_frame": base_link_frame,
            "show_ar_tag": show_ar_tag,
            "show_gripper_bar": show_gripper_bar,
            "show_gripper_fingers": show_gripper_fingers,
            "use_world_frame": use_world_frame,
            "external_urdf_loc": external_urdf_loc,
            "use_rviz": use_rviz,
        }.items(),
    )

    sdk = Node(
        condition=UnlessCondition(use_sim),
        package="interbotix_xs_sdk",
        executable="xs_sdk",
        name="xs_sdk",
        namespace=robot_name,
        arguments=[],
        parameters=[
            {
                "motor_configs": motor_configs,
                "mode_configs": mode_configs,
                "load_configs": load_configs,
                "robot_description": model,
            }
        ],
        output="screen",
    )
    sdk_sim = Node(
        condition=IfCondition(use_sim),
        package="interbotix_xs_sdk",
        executable="xs_sdk_sim",
        name="xs_sdk_sim",
        namespace=robot_name,
        arguments=[],
        parameters=[
            {
                "motor_configs": motor_configs,
                "mode_configs": mode_configs,
                "robot_description": model,
            }
        ],
        output="screen",
    )

    nodes_to_start = [sdk, sdk_sim, description]
    return nodes_to_start

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_model",
            default_value='""',
            description="model type of the Interbotix Arm such as 'wx200' or 'rx150'",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value='""',
            description="name of the robot (typically equal to robot_model, but could be anything)'",
        ),
        DeclareLaunchArgument(
            "base_link_frame",
            default_value="'base_link'",
            description="name of the 'root' link on the arm; typically 'base_link', but can be changed if attaching the arm to a mobile base that already has a 'base_link' frame",
        ),
        DeclareLaunchArgument(
            "show_ar_tag",
            default_value="false",
            description="if true, the AR tag mount is included in the 'robot_description' parameter; if false, it is left out; set to true if using the AR tag mount in your project",
        ),
        DeclareLaunchArgument(
            "show_gripper_bar",
            default_value="true",
            description="if true, the gripper_bar link is included in the 'robot_description' parameter; if false, the gripper_bar and finger links are not loaded to the parameter server. Set to false if you have a custom gripper attachment",
        ),
        DeclareLaunchArgument(
            "show_gripper_fingers",
            default_value="true",
            description="if true, the gripper fingers are included in the 'robot_description' parameter; if false, the gripper finger links are not loaded to the parameter server. Set to false if you have custom gripper fingers",
        ),
        DeclareLaunchArgument(
            "use_world_frame",
            default_value="true",
            description="set this to true if you would like to load a 'world' frame to the 'robot_description' parameter which is located exactly at the 'base_link' frame of the robot; if using multiple robots or if you would like to attach the 'base_link' frame of the robot to a different frame, set this to false",
        ),
        DeclareLaunchArgument(
            "external_urdf_loc",
            default_value='""',
            description="the file path to the custom urdf.xacro file that you would like to include in the Interbotix robot's urdf.xacro file",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="launches Rviz",
        ),
        DeclareLaunchArgument(
            "load_gazebo_configs",
            default_value="false",
            description="set this to true if Gazebo is being used; it makes sure to include Gazebo related configs in the 'robot_description' parameter so that the robot models show up black in Gazebo",
        ),
        DeclareLaunchArgument(
            "rvizconfig",
            default_value='""',
            description="file path to the config file Rviz should load",
        ),
        DeclareLaunchArgument(
            "model",
            default_value='""',
            description="file path to the robot-specific URDF including arguments to be passed in",
        ),
        DeclareLaunchArgument(
            "motor_configs",
            default_value='""',
            description="the file path to the 'motor config' YAML file",
        ),
        DeclareLaunchArgument(
            "mode_configs",
            default_value='""',
            description="the file path to the 'mode config' YAML file",
        ),
        DeclareLaunchArgument(
            "load_configs",
            default_value="true",
            description="a boolean that specifies whether or not the initial register values (under the 'motors' heading) in a Motor Config file should be written to the motors; as the values being written are stored in each motor's EEPROM (which means the values are retained even after a power cycle), this can be set to false after the first time using the robot. Setting to false also shortens the node startup time by a few seconds and preserves the life of the EEPROM",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="if true, the Dynamixel simulator node is run; use Rviz to visualize the robot's motion; if false, the real Dynamixel driver node is run",
        ),
        OpaqueFunction(function = launch_setup)
        ])