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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_model_1_launch_arg = LaunchConfiguration('robot_model_1')
    robot_name_1_launch_arg = LaunchConfiguration('robot_name_1')
    mode_configs_1_launch_arg = LaunchConfiguration('mode_configs_1')
    robot_description_1_launch_arg = LaunchConfiguration('robot_description_1')

    robot_model_2_launch_arg = LaunchConfiguration('robot_model_2')
    robot_name_2_launch_arg = LaunchConfiguration('robot_name_2')
    mode_configs_2_launch_arg = LaunchConfiguration('mode_configs_2')
    robot_description_2_launch_arg = LaunchConfiguration('robot_description_2')

    use_sim_launch_arg = LaunchConfiguration('use_sim')

    xsarm_control_1_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_1_launch_arg,
            'robot_name': robot_name_1_launch_arg,
            'mode_configs': mode_configs_1_launch_arg,
            'use_rviz': 'false',
            'robot_description': robot_description_1_launch_arg,
            'use_sim': use_sim_launch_arg,
        }.items(),
    )

    xsarm_control_2_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_2_launch_arg,
            'robot_name': robot_name_2_launch_arg,
            'mode_configs': mode_configs_2_launch_arg,
            'use_rviz': 'false',
            'robot_description': robot_description_2_launch_arg,
            'use_sim': use_sim_launch_arg,
        }.items(),
    )

    robot_1_transform_broadcaster_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_1_transform_broadcaster',
        arguments=[
            '0.0',
            '-0.25',
            '0.0',
            '0.0',
            '0.0',
            '0.0',
            '1.0',
            '/world',
            (
                '/', LaunchConfiguration('robot_name_1'),
                '/', LaunchConfiguration('base_link_frame')
            ),
        ],
    )

    robot_2_transform_broadcaster_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_2_transform_broadcaster',
        arguments=[
            '0.0',
            '0.25',
            '0.0',
            '0.0',
            '0.0',
            '0.0',
            '1.0',
            '/world',
            (
                '/', LaunchConfiguration('robot_name_2'),
                '/', LaunchConfiguration('base_link_frame')
            ),
        ],
        output={'both': 'screen'}
    )

    dual_rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_dual_rviz')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('dual_rviz_config')],
        output={'both': 'log'},
    )

    return [
        xsarm_control_1_launch_include,
        xsarm_control_2_launch_include,
        robot_1_transform_broadcaster_node,
        robot_2_transform_broadcaster_node,
        dual_rviz_node,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model_1',
            default_value='wx200',
            choices=get_interbotix_xsarm_models(),
            description='model type of the first Interbotix Arm such as `wx200` or `rx150`.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name_1',
            default_value='arm_1',
            description='name of the first robot',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs_1',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_dual'),
                'config',
                'modes_1.yaml',
            ]),
            description="the file path to the 'mode config' YAML file for the first arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model_2',
            default_value='wx200',
            choices=get_interbotix_xsarm_models(),
            description='model type of the second Interbotix Arm such as `wx200` or `rx150`.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name_2',
            default_value='arm_2',
            description='name of the second robot',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs_2',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_dual'),
                'config',
                'modes_2.yaml',
            ]),
            description="the file path to the 'mode config' YAML file for the second arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_dual_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='launches RViz for the dual-arm configuration if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'dual_rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_dual'),
                'rviz',
                'xsarm_dual.rviz',
            ]),
            description='file path to the dual-arm config file RViz should load.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'if `true`, the DYNAMIXEL simulator node is run; use RViz to visualize the'
                " robot's motion; if `false`, the real DYNAMIXEL driver node is run."
            ),
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            robot_description_launch_config_name='robot_description_1',
            robot_model_launch_config_name='robot_model_1',
            robot_name_launch_config_name='robot_name_1',
            base_link_frame='base_link',
            use_world_frame='false',
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            robot_description_launch_config_name='robot_description_2',
            robot_model_launch_config_name='robot_model_2',
            robot_name_launch_config_name='robot_name_2',
            base_link_frame='base_link',
            use_world_frame='false',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
