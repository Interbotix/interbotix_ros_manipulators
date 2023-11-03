import os

from ament_index_python.packages import get_package_share_directory

from interbotix_xs_modules.xs_common import (
    get_interbotix_xscobot_models,
)

from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xscobot_robot_description_launch_arguments,
    determine_use_sim_time_param,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

import yaml

"""
A launch file for running the motion planning python api tutorial
"""


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    base_link_frame_launch_arg = LaunchConfiguration('base_link_frame')
    use_world_frame_launch_arg = LaunchConfiguration('use_world_frame')
    show_ar_tag_launch_arg = LaunchConfiguration('show_ar_tag')
    external_urdf_loc_launch_arg = LaunchConfiguration('external_urdf_loc')
    mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    rviz_frame_launch_arg = LaunchConfiguration('rviz_frame')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')

    use_sim_time_param = determine_use_sim_time_param(
        context=context, hardware_type_launch_arg=hardware_type_launch_arg
    )

    remappings = [
        (
            f'{robot_name_launch_arg.perform(context)}/get_planning_scene',
            f'/{robot_name_launch_arg.perform(context)}/get_planning_scene',
        ),
        (
            '/arm_controller/follow_joint_trajectory',
            f'/{robot_name_launch_arg.perform(context)}'
            '/arm_controller/follow_joint_trajectory',
        ),
        (
            '/gripper_controller/follow_joint_trajectory',
            f'/{robot_name_launch_arg.perform(context)}'
            '/gripper_controller/follow_joint_trajectory',
        ),
    ]

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='dx400', package_name='interbotix_xscobot_moveit'
        )
        .robot_description(file_path='config/dx400.urdf.xacro')
        .robot_description_semantic(file_path='config/srdf/dx400.srdf.xacro')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .joint_limits(file_path='config/joint_limits/dx400_joint_limits.yaml')
        .robot_description_kinematics(file_path='config/moveit_kinematics.yaml')
        .planning_scene_monitor(planning_scene_monitor_parameters)
        # .planning_pipelines(pipelines=['ompl'])
        .moveit_cpp(
            file_path=get_package_share_directory('interbotix_moveit_interface')
            + '/config/planning.yaml'
        )
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .to_moveit_configs()
    )

    example_file = DeclareLaunchArgument(
        'example_file',
        default_value='example.py',
        description='Python API tutorial file name',
    )

    moveit_py_node = Node(
        name='moveit_py',
        package='interbotix_xscobot_moveit',
        executable=LaunchConfiguration('example_file'),
        output='both',
        parameters=[moveit_config.to_dict()],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('interbotix_xsarm_moveit'),
        'rviz',
        'xsarm_moveit.rviz',
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        # namespace=robot_name_launch_arg,
        parameters=[
            {
                'planning_scene_monitor_options': {
                    'robot_description': 'robot_description',
                    'joint_state_topic': f'/{robot_name_launch_arg.perform(context)}/joint_states',
                },
                'use_sim_time': use_sim_time_param,
            },
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.trajectory_execution,
        ],
        remappings=remappings,
        output={'both': 'screen'},
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output={'both': 'log'},
        arguments=[
            '-d',
            rviz_config_file,
            '-f',
            rviz_frame_launch_arg,
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        remappings=remappings,
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=[
            '0.0',
            '0.0',
            '0.0',
            '0.0',
            '0.0',
            '0.0',
            'world',
            'dx400/base_link',
        ],
    )

    xscobot_ros_control_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('interbotix_xscobot_ros_control'),
                        'launch',
                        'xscobot_ros_control.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch_arg,
            'base_link_frame': base_link_frame_launch_arg,
            'show_gripper_fingers': 'true',
            'show_ar_tag': show_ar_tag_launch_arg,
            'use_world_frame': use_world_frame_launch_arg,
            'external_urdf_loc': external_urdf_loc_launch_arg,
            'use_rviz': 'false',
            'mode_configs': mode_configs_launch_arg,
            'hardware_type': hardware_type_launch_arg,
            'robot_description': robot_description_launch_arg,
            'use_sim_time': use_sim_time_param,
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", hardware_type_launch_arg, "'", ' in ("actual", "fake")']
            )
        ),
    )
    return [
        TimerAction(
                period=5.0,
                actions=[moveit_py_node],
                    ),
        example_file,
        move_group_node,
        xscobot_ros_control_launch_include,
        rviz_node,
        static_tf,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            choices=get_interbotix_xscobot_models(),
            description='model type of the Interbotix Cobot such as `dx400`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description=(
                'name of the robot (typically equal to `robot_model`, but could be anything).'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'external_srdf_loc',
            default_value=TextSubstitution(text=''),
            description=(
                'the file path to the custom semantic description file that you would like to '
                "include in the Interbotix robot's semantic description."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('interbotix_xscobot_moveit'),
                    'config',
                    'modes.yaml',
                ]
            ),
            description="the file path to the 'mode config' YAML file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'xs_driver_logging_level',
            default_value='INFO',
            choices=('DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'),
            description='set the logging level of the X-Series Driver.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_moveit_rviz',
            default_value='true',
            choices=('true', 'false'),
            description="launches RViz with MoveIt's RViz configuration",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_frame',
            default_value='world',
            description=(
                'defines the fixed frame parameter in RViz. Note that if `use_world_frame` is '
                '`false`, this parameter should be changed to a frame that exists.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('interbotix_xsarm_moveit'),
                    'rviz',
                    'xsarm_moveit.rviz',
                ]
            ),
            description='file path to the config file RViz should load.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'world_filepath',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('interbotix_common_sim'),
                    'worlds',
                    'interbotix.world',
                ]
            ),
            description="the file path to the Gazebo 'world' file to load.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                'published over the ROS topic /clock; this value is automatically set to `true` if'
                ' using Gazebo hardware.'
            ),
        )
    )
    declared_arguments.extend(
        declare_interbotix_xscobot_robot_description_launch_arguments(
            show_gripper_fingers='true',
            hardware_type='actual',
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
