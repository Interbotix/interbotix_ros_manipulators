import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
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


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context, *args, **kwargs):

    robot_model_launch_arg = LaunchConfiguration("robot_model")
    robot_name_launch_arg = LaunchConfiguration("robot_name")
    base_link_frame_launch_arg = LaunchConfiguration("base_link_frame")
    show_ar_tag_launch_arg = LaunchConfiguration("show_ar_tag")
    use_world_frame_launch_arg = LaunchConfiguration("use_world_frame")
    external_urdf_loc_launch_arg = LaunchConfiguration("external_urdf_loc")
    external_srdf_loc_launch_arg = LaunchConfiguration("external_srdf_loc")
    mode_configs_launch_arg = LaunchConfiguration("mode_configs")
    use_moveit_rviz_launch_arg = LaunchConfiguration("use_moveit_rviz")
    rviz_frame_launch_arg = LaunchConfiguration("rviz_frame")
    use_gazebo_launch_arg = LaunchConfiguration("use_gazebo")
    use_actual_launch_arg = LaunchConfiguration("use_actual")
    use_fake_launch_arg = LaunchConfiguration("use_fake")
    dof_launch_arg = LaunchConfiguration("dof")
    world_name_launch_arg = LaunchConfiguration("world_name")
    load_gazebo_configs_launch_arg = LaunchConfiguration("load_gazebo_configs")
    model_launch_arg = LaunchConfiguration("model")

    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("interbotix_xsarm_descriptions"),
            "urdf",
            robot_model_launch_arg,
        ]
    )

    model_launch_arg = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            urdf_path, ".urdf.xacro", " ",
            "robot_name:=",             robot_name_launch_arg, " ",
            "base_link_frame:=",        base_link_frame_launch_arg, " ",
            "show_ar_tag:=",            show_ar_tag_launch_arg, " ",
            "show_gripper_bar:=",       "true", " ",
            "show_gripper_fingers:=",   "true", " ",
            "use_world_frame:=",        use_world_frame_launch_arg, " ",
            "external_urdf_loc:=",      external_urdf_loc_launch_arg, " ",
            "load_gazebo_configs:=",    load_gazebo_configs_launch_arg, " ",
        ]
    )

    robot_description = {"robot_description": model_launch_arg}

    config_path = PathJoinSubstitution(
        [
            FindPackageShare("interbotix_xsarm_moveit"),
            "config",
        ]
    )

    robot_description_semantic_config = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            config_path, f"/srdf/{robot_model_launch_arg.perform(context)}.srdf.xacro", " ",
            "robot_name:=",             robot_name_launch_arg, " ",
            "base_link_frame:=",        base_link_frame_launch_arg, " ",
            "show_ar_tag:=",            show_ar_tag_launch_arg, " ",
            "show_gripper_bar:=",       "true", " ",
            "show_gripper_fingers:=",   "true", " ",
            "use_world_frame:=",        use_world_frame_launch_arg, " ",
            "external_urdf_loc:=",      external_urdf_loc_launch_arg, " ",
            "external_srdf_loc:=",      external_srdf_loc_launch_arg, " ",
            "load_gazebo_configs:=",    load_gazebo_configs_launch_arg, " ",
        ]
    )

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # kinematics_config = load_yaml("interbotix_xsarm_moveit", "config/kinematics.yaml")
    kinematics_config = PathJoinSubstitution(
        [
            FindPackageShare("interbotix_xsarm_moveit"),
            "config",
            "kinematics.yaml",
        ]
    )

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                                "default_planner_request_adapters/FixWorkspaceBounds "
                                "default_planner_request_adapters/FixStartStateBounds "
                                "default_planner_request_adapters/FixStartStateCollision "
                                "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }

    # ompl_planning_yaml = PathJoinSubstitution(
    #     [
    #         FindPackageShare("interbotix_xsarm_moveit"), "config/ompl_planning.yaml"
    #     ]
    # )

    ompl_planning_pipeline_yaml_file = load_yaml(
        "interbotix_xsarm_moveit", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_pipeline_yaml_file)

    # ompl_planning_pipeline_config = PathJoinSubstitution(
    #     [
    #         FindPackageShare("interbotix_xsarm_moveit"),
    #         "config",
    #         "ompl_planning.yaml",
    #     ]
    # )

    controllers_config = load_yaml(
        "interbotix_xsarm_moveit",
        f"config/controllers/{dof_launch_arg.perform(context)}dof_controllers.yaml"
    )
    # controllers_config = PathJoinSubstitution(
    #     [
    #         FindPackageShare("interbotix_xsarm_moveit"),
    #         "config",
    #         "controllers",
    #         f"{dof_launch_arg.perform(context)}dof_controllers.yaml",
    #     ]
    # )

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_config,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution_parameters = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 4.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    sensor_parameters = {"sensors": [""],}

    remappings = [
        (f"{robot_name_launch_arg.perform(context)}/get_planning_scene",
            f"/{robot_name_launch_arg.perform(context)}/get_planning_scene"),
        (f"/arm_controller/follow_joint_trajectory",
            f"/{robot_name_launch_arg.perform(context)}/arm_controller/follow_joint_trajectory")
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output={"both": "screen"},
        # namespace=robot_name_launch_arg,
        parameters=[
            {
                "planning_scene_monitor_options": {
                    "robot_description":
                        "robot_description",
                    "joint_state_topic":
                        f"/{robot_name_launch_arg.perform(context)}/joint_states",
                }
            },
            robot_description,
            robot_description_semantic,
            kinematics_config,
            ompl_planning_pipeline_config,
            trajectory_execution_parameters,
            moveit_controllers,
            planning_scene_monitor_parameters,
            # sensor_parameters,
        ],
        remappings=remappings,
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("interbotix_xsarm_moveit"), "rviz", "xsarm_moveit.rviz",]
    )

    moveit_rviz_node = Node(
        condition=IfCondition(use_moveit_rviz_launch_arg),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # namespace=robot_name_launch_arg,
        output={"both": "screen"},
        arguments=[
            "-d", rviz_config_file,
            "-f", rviz_frame_launch_arg,
            ],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_config,
        ],
        remappings=remappings,
    )

    xsarm_ros_control_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("interbotix_xsarm_ros_control"),
                        "launch",
                        "xsarm_ros_control.launch.py"
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_model": robot_model_launch_arg,
            "robot_name": robot_name_launch_arg,
            "base_link_frame": base_link_frame_launch_arg,
            "show_ar_tag": show_ar_tag_launch_arg,
            "show_gripper_bar": "true",
            "show_gripper_fingers": "true",
            "use_world_frame": use_world_frame_launch_arg,
            "external_urdf_loc": external_urdf_loc_launch_arg,
            "use_rviz": "false",
            "use_sim": "false",
        }.items(),
        condition=IfCondition(use_actual_launch_arg),
    )

    return [
        move_group_node,
        moveit_rviz_node,
        xsarm_ros_control_launch_include,
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
                "external_srdf_loc",
                default_value=TextSubstitution(text=""),
                description=(
                    "the file path to the custom semantic description file that you "
                    "would like to include in the Interbotix robot's semantic ""description"
                ),
            ),
            DeclareLaunchArgument(
                "mode_configs",
                default_value=TextSubstitution(text=""),
                description="the file path to the 'mode config' YAML file",
            ),
            DeclareLaunchArgument(
                "use_moveit_rviz",
                default_value="true",
                description="launches RViz with MoveIt's RViz configuration",
            ),
            DeclareLaunchArgument(
                "rviz_frame",
                default_value="world",
            ),
            DeclareLaunchArgument(
                "use_gazebo",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "use_actual",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "use_fake",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "dof",
                default_value="5",
                choices=["4", "5", "6"],
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value=TextSubstitution(text=""),
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