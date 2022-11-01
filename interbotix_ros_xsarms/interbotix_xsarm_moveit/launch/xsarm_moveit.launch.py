# Copyright 2022 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory
from interbotix_xs_modules.xs_common import (
    get_interbotix_xsarm_models,
)
from interbotix_xs_modules.xs_launch import (
    construct_interbotix_xsarm_semantic_robot_description_command,
    declare_interbotix_xsarm_robot_description_launch_arguments,
    determine_use_sim_time_param,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    base_link_frame_launch_arg = LaunchConfiguration('base_link_frame')
    show_ar_tag_launch_arg = LaunchConfiguration('show_ar_tag')
    use_world_frame_launch_arg = LaunchConfiguration('use_world_frame')
    external_urdf_loc_launch_arg = LaunchConfiguration('external_urdf_loc')
    mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    use_moveit_rviz_launch_arg = LaunchConfiguration('use_moveit_rviz')
    rviz_frame_launch_arg = LaunchConfiguration('rviz_frame')
    rviz_config_file_launch_arg = LaunchConfiguration('rviz_config_file')
    world_filepath_launch_arg = LaunchConfiguration('world_filepath')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')

    # sets use_sim_time parameter to 'true' if using gazebo hardware
    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        hardware_type_launch_arg=hardware_type_launch_arg
    )

    robot_description = {'robot_description': robot_description_launch_arg}

    config_path = PathJoinSubstitution([
        FindPackageShare('interbotix_xsarm_moveit'),
        'config',
    ])

    robot_description_semantic = {
        'robot_description_semantic':
            construct_interbotix_xsarm_semantic_robot_description_command(
                robot_model=robot_model_launch_arg.perform(context),
                config_path=config_path
            ),
    }

    kinematics_config = PathJoinSubstitution([
        FindPackageShare('interbotix_xsarm_moveit'),
        'config',
        'kinematics.yaml',
    ])

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin':
                'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error':
                0.1,
        }
    }

    ompl_planning_pipeline_yaml_file = load_yaml(
        'interbotix_xsarm_moveit', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_pipeline_yaml_file)

    controllers_config = load_yaml(
        'interbotix_xsarm_moveit',
        f'config/controllers/{robot_model_launch_arg.perform(context)}_controllers.yaml'
    )

    config_joint_limits = load_yaml(
        'interbotix_xsarm_moveit',
        f'config/joint_limits/{robot_model_launch_arg.perform(context)}_joint_limits.yaml'
    )

    joint_limits = {
        'robot_description_planning': config_joint_limits,
    }

    moveit_controllers = {
        'moveit_simple_controller_manager':
            controllers_config,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution_parameters = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    sensor_parameters = {
        'sensors': [''],
    }

    remappings = [
        (
            f'{robot_name_launch_arg.perform(context)}/get_planning_scene',
            f'/{robot_name_launch_arg.perform(context)}/get_planning_scene'
        ),
        (
            '/arm_controller/follow_joint_trajectory',
            f'/{robot_name_launch_arg.perform(context)}/arm_controller/follow_joint_trajectory'
        ),
        (
            '/gripper_controller/follow_joint_trajectory',
            f'/{robot_name_launch_arg.perform(context)}/gripper_controller/follow_joint_trajectory'
        ),
    ]

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        # namespace=robot_name_launch_arg,
        parameters=[
            {
                'planning_scene_monitor_options': {
                    'robot_description':
                        'robot_description',
                    'joint_state_topic':
                        f'/{robot_name_launch_arg.perform(context)}/joint_states',
                },
                'use_sim_time': use_sim_time_param,
            },
            robot_description,
            robot_description_semantic,
            kinematics_config,
            ompl_planning_pipeline_config,
            trajectory_execution_parameters,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits,
            sensor_parameters,
        ],
        remappings=remappings,
        output={'both': 'screen'},
    )

    moveit_rviz_node = Node(
        condition=IfCondition(use_moveit_rviz_launch_arg),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # namespace=robot_name_launch_arg,
        arguments=[
            '-d', rviz_config_file_launch_arg,
            '-f', rviz_frame_launch_arg,
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_config,
            {'use_sim_time': use_sim_time_param},
        ],
        remappings=remappings,
        output={'both': 'log'},
    )

    xsarm_ros_control_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_ros_control'),
                'launch',
                'xsarm_ros_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch_arg,
            'base_link_frame': base_link_frame_launch_arg,
            'show_ar_tag': show_ar_tag_launch_arg,
            'show_gripper_bar': 'true',
            'show_gripper_fingers': 'true',
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
                ['"', hardware_type_launch_arg, '"', " in ('actual', 'fake')"]
            )
        ),
    )

    xsarm_gz_classic_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_sim'),
                'launch',
                'xsarm_gz_classic.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch_arg,
            'base_link_frame': base_link_frame_launch_arg,
            'show_ar_tag': show_ar_tag_launch_arg,
            'show_gripper_bar': 'true',
            'show_gripper_fingers': 'true',
            'use_world_frame': use_world_frame_launch_arg,
            'external_urdf_loc': external_urdf_loc_launch_arg,
            'use_rviz': 'false',
            'world_filepath': world_filepath_launch_arg,
            'hardware_type': hardware_type_launch_arg,
            'robot_description': robot_description_launch_arg,
            'use_sim_time': use_sim_time_param,
        }.items(),
        condition=LaunchConfigurationEquals(
            launch_configuration_name='hardware_type',
            expected_value='gz_classic'
        ),
    )

    return [
        move_group_node,
        moveit_rviz_node,
        xsarm_ros_control_launch_include,
        xsarm_gz_classic_launch_include,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            choices=get_interbotix_xsarm_models(),
            description='model type of the Interbotix Arm such as `wx200` or `rx150`.',
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
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit'),
                'config',
                'modes.yaml',
            ]),
            description="the file path to the 'mode config' YAML file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'xs_driver_logging_level',
            default_value='INFO',
            choices=('DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'),
            description='set the logging level of the X-Series Driver.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_moveit_rviz',
            default_value='true',
            choices=('true', 'false'),
            description="launches RViz with MoveIt's RViz configuration/",
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
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit'),
                'rviz',
                'xsarm_moveit.rviz'
            ]),
            description='file path to the config file RViz should load.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'world_filepath',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_common_sim'),
                'worlds',
                'interbotix.world',
            ]),
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
            )
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            show_gripper_bar='true',
            show_gripper_fingers='true',
            hardware_type='actual',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
