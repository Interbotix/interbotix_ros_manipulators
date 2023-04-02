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

from pathlib import Path

from interbotix_xs_modules.xs_common import (
    get_interbotix_xsarm_models,
)
from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
from interbotix_xs_modules.xs_launch.xs_launch import determine_use_sim_time_param
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    rviz_config_launch_arg = LaunchConfiguration('rvizconfig')
    world_filepath_launch_arg = LaunchConfiguration('world_filepath')
    use_gazebo_gui_launch_arg = LaunchConfiguration('use_gazebo_gui')
    verbose_launch_arg = LaunchConfiguration('verbose')
    debug_launch_arg = LaunchConfiguration('debug')
    paused_launch_arg = LaunchConfiguration('paused')
    recording_launch_arg = LaunchConfiguration('recording')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')

    # sets use_sim_time parameter to 'true' if using gazebo hardware
    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        hardware_type_launch_arg=hardware_type_launch_arg
    )

    # Set ignition resource path
    gz_resource_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            '/usr/share/gazebo-11/models/',
            ':',
            str(Path(
                FindPackageShare('interbotix_common_sim').perform(context)
            ).parent.resolve()),
            ':',
            str(Path(
                FindPackageShare('interbotix_xsarm_descriptions').perform(context)
            ).parent.resolve()),
        ]
    )

    # Set GAZEBO_MODEL_URI to empty string to prevent Gazebo from downloading models
    gz_model_uri_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_URI',
        value=['']
    )

    # Set GAZEBO_MODEL_DATABASE_URI to empty string to prevent Gazebo from downloading models
    gz_model_uri_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=['']
    )

    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ]),
        ]),
        launch_arguments={
            'verbose': verbose_launch_arg,
            'world': world_filepath_launch_arg,
            'pause': paused_launch_arg,
            'record': recording_launch_arg,
            'gdb': debug_launch_arg,
            'valgrind': debug_launch_arg,
            'gui': use_gazebo_gui_launch_arg,
        }.items(),
    )

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_{robot_name_launch_arg.perform(context)}',
        arguments=[
            '-entity', 'robot_description',
            '-topic', f'{robot_name_launch_arg.perform(context)}/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
        ],
        output={'both': 'log'},
    )

    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            '-c',
            'controller_manager',
            'joint_state_broadcaster',
        ],
        parameters=[{
            'use_sim_time': use_sim_time_param,
        }],
    )

    spawn_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            '-c',
            'controller_manager',
            'arm_controller',
        ],
        parameters=[{
            'use_sim_time': use_sim_time_param,
        }]
    )

    spawn_gripper_controller_node = Node(
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            '-c',
            'controller_manager',
            'gripper_controller',
        ],
        parameters=[{
            'use_sim_time': use_sim_time_param,
        }]
    )

    xsarm_description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_descriptions'),
                'launch',
                'xsarm_description.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch_arg,
            'use_rviz': use_rviz_launch_arg,
            'rvizconfig': rviz_config_launch_arg,
            'use_sim_time': use_sim_time_param,
            'robot_description': robot_description_launch_arg,
        }.items(),
    )

    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[spawn_joint_state_broadcaster_node]
        )
    )

    load_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_arm_controller_node]
        )
    )

    load_gripper_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_gripper_controller_node]
        )
    )

    return [
        gz_resource_path_env_var,
        gz_model_uri_env_var,
        gazebo_launch_include,
        spawn_robot_node,
        load_joint_state_broadcaster_event,
        load_arm_controller_event,
        load_gripper_controller_event,
        xsarm_description_launch_include,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            choices=get_interbotix_xsarm_models(),
            description='model type of the Interbotix Arm such as `wx200` or `rx150`.'
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
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_sim'),
                'rviz',
                'xsarm_gz_classic.rviz',
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
            'use_gazebo_gui',
            default_value='true',
            choices=('true', 'false'),
            description='launches the Gazebo GUI if `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            choices=('true', 'false'),
            description='launches Gazebo with verbose console logging if `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            choices=('true', 'false'),
            description='start gzserver in debug mode using gdb.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'paused',
            default_value='false',
            choices=('true', 'false'),
            description='start Gazebo in a paused state.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'recording',
            default_value='false',
            choices=('true', 'false'),
            description='enable Gazebo state log recording.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                'published over the ROS topic /clock.'
            )
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            hardware_type='gz_classic',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
