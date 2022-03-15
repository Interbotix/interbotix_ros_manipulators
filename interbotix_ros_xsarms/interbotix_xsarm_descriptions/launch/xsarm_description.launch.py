from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


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
    load_gazebo_configs = LaunchConfiguration("load_gazebo_configs")
    use_joint_pub = LaunchConfiguration("use_joint_pub")
    use_joint_pub_gui = LaunchConfiguration("use_joint_pub_gui")
    rvizconfig = LaunchConfiguration("rvizconfig")
    model = LaunchConfiguration("model")

    xsarm_descriptions_prefix = get_package_share_directory(
        "interbotix_xsarm_descriptions"
    )

    rvizconfig = PathJoinSubstitution(
        [xsarm_descriptions_prefix, "rviz", "xsarm_description.rviz"]
    )

    urdf_path = PathJoinSubstitution([xsarm_descriptions_prefix, "urdf", robot_model])

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

    robot_description = {"robot_description": model}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        namespace=robot_name,
        output={"both": "log"},
    )

    joint_state_publisher_node = Node(
        condition=IfCondition(use_joint_pub),
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace=robot_name,
        output={"both": "log"},
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(use_joint_pub_gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        namespace=robot_name,
        output={"both": "log"},
    )

    rviz2_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=robot_name,
        arguments=["-d", rvizconfig],
        output={"both": "log"},
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value=TextSubstitution(text=""),
                description=(
                    "model type of the Interbotix Arm such as 'wx200' or 'rx150'"
                ),
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value=LaunchConfiguration("robot_model"),
                description=(
                    "name of the robot (typically equal to robot_model, but could be "
                    "anything)'"
                ),
            ),
            DeclareLaunchArgument(
                "base_link_frame",
                default_value=TextSubstitution(text="base_link"),
                description=(
                    "name of the 'root' link on the arm; typically 'base_link', but "
                    "can be changed if attaching the arm to a mobile base that "
                    "already has a 'base_link' frame"
                ),
            ),
            DeclareLaunchArgument(
                "show_ar_tag",
                default_value="false",
                description=(
                    "if true, the AR tag mount is included in the 'robot_description'"
                    " parameter; if false, it is left out; set to true if using the "
                    "AR tag mount in your project"
                ),
            ),
            DeclareLaunchArgument(
                "show_gripper_bar",
                default_value="true",
                description=(
                    "if true, the gripper_bar link is included in the "
                    "'robot_description' parameter; if false, the gripper_bar and "
                    "finger links are not loaded. Set to false if you have a custom "
                    "gripper attachment"
                ),
            ),
            DeclareLaunchArgument(
                "show_gripper_fingers",
                default_value="true",
                description=(
                    "if true, the gripper fingers are included in the "
                    "'robot_description' parameter; if false, the gripper "
                    "finger links are not loaded. Set to false if you have "
                    "custom gripper fingers"
                ),
            ),
            DeclareLaunchArgument(
                "use_world_frame",
                default_value="true",
                description=(
                    "set this to true if you would like to load a 'world' frame "
                    "to the 'robot_description' parameter which is located exactly "
                    "at the 'base_link' frame of the robot; if using multiple "
                    "robots or if you would like to attach the 'base_link' frame of "
                    "the robot to a different frame, set this to false"
                ),
            ),
            DeclareLaunchArgument(
                "external_urdf_loc",
                default_value=TextSubstitution(text=""),
                description=(
                    "the file path to the custom urdf.xacro file that you would like "
                    "to include in the Interbotix robot's urdf.xacro file"
                ),
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="launches Rviz",
            ),
            DeclareLaunchArgument(
                "load_gazebo_configs",
                default_value="false",
                description=(
                    "set this to true if Gazebo is being used; it makes sure to "
                    "include Gazebo related configs in the 'robot_description' "
                    "parameter so that the robot models show up black in Gazebo"
                ),
            ),
            DeclareLaunchArgument(
                "use_joint_pub",
                default_value="false",
                description="launches the joint_state_publisher node",
            ),
            DeclareLaunchArgument(
                "use_joint_pub_gui",
                default_value="false",
                description="launches the joint_state_publisher GUI",
            ),
            DeclareLaunchArgument(
                "rvizconfig",
                default_value=TextSubstitution(text=""),
                description="file path to the config file Rviz should load",
            ),
            DeclareLaunchArgument(
                "model",
                default_value=TextSubstitution(text=""),
                description=(
                    "file path to the robot-specific URDF including arguments to be "
                    "passed in"
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
