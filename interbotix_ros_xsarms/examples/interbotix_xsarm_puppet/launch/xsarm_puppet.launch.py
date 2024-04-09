# Copyright 2024 Trossen Robotics
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
#    * Neither the name of the the copyright holder nor the names of its
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

from typing import List

from interbotix_xs_modules.xs_common import (
    get_interbotix_xsarm_models,
)
from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import (
  IfCondition,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_model_leader_launch_arg = LaunchConfiguration('robot_model_leader')
    robot_name_leader_launch_arg = LaunchConfiguration('robot_name_leader')
    robot_model_follower_launch_arg = LaunchConfiguration('robot_model_follower')
    robot_name_follower_launch_arg = LaunchConfiguration('robot_name_follower')
    use_puppet_rviz_launch_arg = LaunchConfiguration('use_puppet_rviz')
    launch_driver_launch_arg = LaunchConfiguration('launch_driver')
    use_sim_launch_arg = LaunchConfiguration('use_sim')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')

    xsarm_control_launch_leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_leader_launch_arg,
            'robot_name': robot_name_leader_launch_arg,
            'base_link_frame': 'base_link',
            'use_rviz': 'false',
            'mode_configs': LaunchConfiguration('mode_configs_leader'),
            'motor_configs': LaunchConfiguration('motor_configs_leader'),
            'use_sim': use_sim_launch_arg,
            'robot_description': LaunchConfiguration('robot_description_leader'),
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }.items(),
        condition=IfCondition(launch_driver_launch_arg),
    )

    xsarm_control_launch_follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_follower_launch_arg,
            'robot_name': robot_name_follower_launch_arg,
            'base_link_frame': 'base_link',
            'use_rviz': 'false',
            'mode_configs': LaunchConfiguration('mode_configs_follower'),
            'motor_configs': LaunchConfiguration('motor_configs_follower'),
            'use_sim': use_sim_launch_arg,
            'robot_description': LaunchConfiguration('robot_description_follower'),
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }.items(),
        condition=IfCondition(launch_driver_launch_arg),
    )

    xsarm_puppet_node = Node(
        name='xsarm_puppet',
        package='interbotix_xsarm_puppet',
        executable='xsarm_puppet',
        output='screen',
        parameters=[
            {
                'robot_name_leader': LaunchConfiguration('robot_name_leader'),
                'robot_name_follower': LaunchConfiguration('robot_name_follower'),
            }
        ]
    )

    tf_broadcaster_leader = Node(
        name='tf_broadcaster_leader',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.0',
            '--y', '-0.25',
            '--z', '0.0',
            '--frame-id', '/world',
            '--child-frame-id', (robot_name_leader_launch_arg, '/base_link'),
        ]
    )

    tf_broadcaster_follower = Node(
        name='tf_broadcaster_follower',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.0',
            '--y', '0.25',
            '--z', '0.0',
            '--frame-id', '/world',
            '--child-frame-id', (robot_name_follower_launch_arg, '/base_link'),
        ]
    )

    rviz2_node = Node(
        condition=IfCondition(use_puppet_rviz_launch_arg),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', LaunchConfiguration('rvizconfig'),
        ],
        output={'both': 'log'},
    )

    return [
        xsarm_control_launch_leader,
        xsarm_control_launch_follower,
        xsarm_puppet_node,
        tf_broadcaster_leader,
        tf_broadcaster_follower,
        rviz2_node,
    ]


def generate_launch_description():
    declared_arguments: List[DeclareLaunchArgument] = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model_leader',
            choices=get_interbotix_xsarm_models(),
            description='model type of the leader Interbotix Arm such as `wx200` or `rx150`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name_leader',
            default_value='leader',
            description=(
                'name of the leader robot.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model_follower',
            choices=get_interbotix_xsarm_models(),
            description='model type of the follower Interbotix Arm such as `wx200` or `rx150`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name_follower',
            default_value='follower',
            description=(
                'name of the follower robot.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs_leader',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_puppet'),
                'config',
                'leader_modes.yaml',
            ]),
            description="the file path to the 'mode config' YAML file for the leader arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs_follower',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_puppet'),
                'config',
                'follower_modes.yaml',
            ]),
            description="the file path to the 'mode config' YAML file for the follower arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'motor_configs_leader',
            default_value=(PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'config',
                LaunchConfiguration('robot_model_leader')]), '.yaml'),
            description="the file path to the 'motor config' YAML file for the leader arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'motor_configs_follower',
            default_value=(PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'config',
                LaunchConfiguration('robot_model_follower')]), '.yaml'),
            description="the file path to the 'motor config' YAML file for the follower arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_puppet_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_puppet'),
                'rviz',
                'xsarm_puppet.rviz',
            ]),
            description='file path to the config file RViz should load.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_driver',
            default_value='true',
            choices=('true', 'false'),
            description=(
                '`true` if xsarm_control should be launched - set to `false` if you would like to '
                'run your own version of this file separately.'
            ),
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
            'use_sim',
            default_value='false',
            choices=('true', 'false'),
            description=(
                "if `true`, the DYNAMIXEL simulator node is run; use RViz to visualize the robot's"
                ' motion; if `false`, the real DYNAMIXEL driver node is run.'
            ),
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            robot_description_launch_config_name='robot_description_leader',
            robot_model_launch_config_name='robot_model_leader',
            robot_name_launch_config_name='robot_name_leader',
            use_world_frame='false',
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            robot_description_launch_config_name='robot_description_follower',
            robot_model_launch_config_name='robot_model_follower',
            robot_name_launch_config_name='robot_name_follower',
            use_world_frame='false',
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
