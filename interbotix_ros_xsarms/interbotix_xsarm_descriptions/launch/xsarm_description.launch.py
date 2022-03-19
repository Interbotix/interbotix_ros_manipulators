from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_model_launch_arg = LaunchConfiguration("robot_model")
    robot_name_launch_arg = LaunchConfiguration("robot_name")
    base_link_frame_launch_arg = LaunchConfiguration("base_link_frame")
    show_ar_tag_launch_arg = LaunchConfiguration("show_ar_tag")
    show_gripper_bar_launch_arg = LaunchConfiguration("show_gripper_bar")
    show_gripper_fingers_launch_arg = LaunchConfiguration("show_gripper_fingers")
    use_world_frame_launch_arg = LaunchConfiguration("use_world_frame")
    external_urdf_loc_launch_arg = LaunchConfiguration("external_urdf_loc")
    use_rviz_launch_arg = LaunchConfiguration("use_rviz")
    load_gazebo_configs_launch_arg = LaunchConfiguration("load_gazebo_configs")
    use_joint_pub_launch_arg = LaunchConfiguration("use_joint_pub")
    use_joint_pub_gui_launch_arg = LaunchConfiguration("use_joint_pub_gui")
    rviz_config_launch_arg = LaunchConfiguration("rvizconfig")
    model_launch_arg = LaunchConfiguration("model")

    rviz_config_launch_arg = PathJoinSubstitution(
        [
            FindPackageShare("interbotix_xsarm_descriptions"), 
            "rviz",
            "xsarm_description.rviz",
        ]
    )

    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("interbotix_xsarm_descriptions"),
            "urdf",
            robot_model_launch_arg
        ]
    )

    model_launch_arg = Command(
        [
            FindExecutable(name="xacro"), " ", urdf_path, ".urdf.xacro ",
            "robot_name:=", robot_name_launch_arg, " ",
            "base_link_frame:=", base_link_frame_launch_arg, " ",
            "show_ar_tag:=", show_ar_tag_launch_arg, " ",
            "show_gripper_bar:=", show_gripper_bar_launch_arg, " ",
            "show_gripper_fingers:=", show_gripper_fingers_launch_arg, " ",
            "use_world_frame:=", use_world_frame_launch_arg, " ",
            "external_urdf_loc:=", external_urdf_loc_launch_arg, " ",
            "load_gazebo_configs:=", load_gazebo_configs_launch_arg, " ",
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": model_launch_arg
            }
        ],
        namespace=robot_name_launch_arg,
        output={"both": "log"},
    )

    joint_state_publisher_node = Node(
        condition=IfCondition(use_joint_pub_launch_arg),
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace=robot_name_launch_arg,
        output={"both": "log"},
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(use_joint_pub_gui_launch_arg),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        namespace=robot_name_launch_arg,
        output={"both": "log"},
    )

    rviz2_node = Node(
        condition=IfCondition(use_rviz_launch_arg),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=robot_name_launch_arg,
        arguments=["-d", rviz_config_launch_arg],
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
                choices=(
                    "px100",
                    "px150",
                    "rx150",
                    "rx200",
                    "wx200",
                    "wx250",
                    "wx250s",
                    "vx250",
                    "vx300",
                    "vx300s",
                    "mobile_px100",
                    "mobile_wx200",
                    "mobile_wx250s",
                ),
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
