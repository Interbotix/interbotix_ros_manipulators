from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    TextSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
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
    load_gazebo_configs_launch_arg = LaunchConfiguration("load_gazebo_configs")
    use_rviz_launch_arg = LaunchConfiguration("use_rviz")
    mode_configs_launch_arg = LaunchConfiguration("mode_configs")
    dof_launch_arg = LaunchConfiguration("dof") # TODO
    model_launch_arg = LaunchConfiguration("model")

    mode_configs_launch_arg = PathJoinSubstitution(
        [
            FindPackageShare("interbotix_xsarm_ros_control"),
            "config",
            "modes.yaml",
        ]
    )

    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("interbotix_xsarm_descriptions"),
            "urdf",
            robot_model_launch_arg,
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

    robot_description = {"robot_description": model_launch_arg}

    controllers_config = PathJoinSubstitution(
        [
            FindPackageShare("interbotix_xsarm_ros_control"),
            "config",
            "controllers",
            f"{robot_model_launch_arg.perform(context)}_controllers.yaml",
        ]
    )

    hardware_config = PathJoinSubstitution(
        [
            FindPackageShare("interbotix_xsarm_ros_control"),
            "config",
            "hardware.yaml",
        ]
    )

    xsarm_control_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("interbotix_xsarm_control"),
                        "launch",
                        "xsarm_control.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_model": robot_model_launch_arg,
            "robot_name": robot_name_launch_arg,
            "base_link_frame": base_link_frame_launch_arg,
            "show_ar_tag": show_ar_tag_launch_arg,
            "show_gripper_bar": show_gripper_bar_launch_arg,
            "show_gripper_fingers": show_gripper_fingers_launch_arg,
            "use_world_frame": use_world_frame_launch_arg,
            "external_urdf_loc": external_urdf_loc_launch_arg,
            "use_rviz": use_rviz_launch_arg,
            "mode_configs": mode_configs_launch_arg,
        }.items(),
    )

    # TODO: load hardware params
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=robot_name_launch_arg,
        parameters=[
            robot_description,
            controllers_config,
            hardware_config,
        ],
        output={"both": "screen"},
    )

    spawn_arm_controller_node = Node(
        name="arm_controller_spawner",
        package="controller_manager",
        executable="spawner",
        namespace=robot_name_launch_arg,
        arguments=[
            "-c",
            f"/{robot_name_launch_arg.perform(context)}/controller_manager",
            "arm_controller",
        ],
    )

    spawn_gripper_controller_node = Node(
        name="gripper_controller_spawner",
        package="controller_manager",
        executable="spawner",
        namespace=robot_name_launch_arg,
        arguments=[
            "-c",
            f"/{robot_name_launch_arg.perform(context)}/controller_manager",
            "gripper_controller",
        ],
    )

    return [
        controller_manager_node,
        spawn_arm_controller_node,
        spawn_gripper_controller_node,
        xsarm_control_launch_include,
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
                default_value="false",
                description="launches RViz",
            ),
            DeclareLaunchArgument(
                "load_gazebo_configs",
                default_value="false",
                description="set this to true if Gazebo is being used; it makes sure to include Gazebo related configs in the 'robot_description' parameter so that the robot models show up black in Gazebo",
            ),
            DeclareLaunchArgument(
                "model",
                default_value='""',
                description="file path to the robot-specific URDF including arguments to be passed in",
            ),
            DeclareLaunchArgument(
                "dof",
                default_value="5",
                description="the degrees of freedom of the arm; while the majority of the arms have 5 dof, others have 4 or 6 dof",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
