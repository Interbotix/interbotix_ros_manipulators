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
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import (
  IfCondition,
  LaunchConfigurationEquals,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    is_record = LaunchConfiguration('record_or_playback').perform(context) == 'record'
    is_playback = LaunchConfiguration('record_or_playback').perform(context) == 'playback'

    if is_record:
        mode_configs = PathJoinSubstitution([
            FindPackageShare('interbotix_xsarm_puppet'),
            'config',
            'record_modes.yaml',
        ])
    elif is_playback:
        mode_configs = PathJoinSubstitution([
            FindPackageShare('interbotix_xsarm_puppet'),
            'config',
            'playback_modes.yaml',
        ])
    else:
        raise ValueError(
            "Launch Argument 'record_or_playback' must be one of ('record', 'playback')"
        )

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    base_link_frame_launch_arg = LaunchConfiguration('base_link_frame')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    launch_driver_launch_arg = LaunchConfiguration('launch_driver')
    use_sim_launch_arg = LaunchConfiguration('use_sim')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')

    puppet_single_node = Node(
        name='xsarm_puppet_single',
        namespace=robot_name_launch_arg,
        package='interbotix_xsarm_puppet',
        executable='xsarm_puppet_single',
        output='screen',
        condition=LaunchConfigurationEquals('record_or_playback', 'record'),
    )

    rosbag_record_node = ExecuteProcess(
        name='rosbag_record_commands',
        cmd=[
            'ros2', 'bag', 'record',
            '--output', PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_puppet'),
                'bag',
                LaunchConfiguration('bag_name'),
            ]),
            (robot_name_launch_arg, '/commands/joint_group'),
            (robot_name_launch_arg, '/commands/joint_single'),
        ],
        condition=LaunchConfigurationEquals('record_or_playback', 'record'),
    )

    rosbag_playback_node = ExecuteProcess(
        name='rosbag_play_commands',
        cmd=[
            'ros2', 'bag', 'play',
            '--delay', '3.0',
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_puppet'),
                'bag',
                LaunchConfiguration('bag_name'),
            ]),
        ],
        condition=LaunchConfigurationEquals('record_or_playback', 'playback'),
    )

    xsarm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch_arg,
            'base_link_frame': base_link_frame_launch_arg,
            'use_rviz': use_rviz_launch_arg,
            'mode_configs': mode_configs,
            'use_sim': use_sim_launch_arg,
            'robot_description': robot_description_launch_arg,
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }.items(),
        condition=IfCondition(launch_driver_launch_arg),
    )

    return [
        puppet_single_node,
        rosbag_record_node,
        rosbag_playback_node,
        xsarm_control_launch,
    ]


def generate_launch_description():
    declared_arguments: List[DeclareLaunchArgument] = []
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
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'record_or_playback',
            choices=('record', 'playback'),
            description=(
                "'record' to record joint commands while physically manipulating the arm to a "
                "bagfile or 'playback' to play-back joint commands from a bagfile to a torqued on "
                'arm.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'bag_name',
            default_value=(LaunchConfiguration('robot_name'), '_commands'),
            description='Desired ROS bag file name.',

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
        declare_interbotix_xsarm_robot_description_launch_arguments()
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
