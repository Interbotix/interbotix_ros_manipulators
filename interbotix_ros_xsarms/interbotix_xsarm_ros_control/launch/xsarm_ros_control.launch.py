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
)
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')

    # sets use_sim_time parameter to 'true' if using gazebo hardware
    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        hardware_type_launch_arg=hardware_type_launch_arg
    )

    ros2_control_controllers_config_parameter_file = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('interbotix_xsarm_ros_control'),
            'config',
            'controllers',
            f'{robot_model_launch_arg.perform(context)}_controllers.yaml',
        ]),
        allow_substs=True
    )

    xsarm_control_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch_arg,
            'use_rviz': use_rviz_launch_arg,
            'mode_configs': mode_configs_launch_arg,
            'hardware_type': hardware_type_launch_arg,
            'use_sim_time': use_sim_time_param,
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }.items(),
        condition=LaunchConfigurationEquals(
            launch_configuration_name='hardware_type',
            expected_value='actual'
        ),
    )

    xsarm_descriptions_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_descriptions'),
                'launch',
                'xsarm_description.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch_arg,
            'use_rviz': use_rviz_launch_arg,
            'mode_configs': mode_configs_launch_arg,
            'hardware_type': hardware_type_launch_arg,
            'use_sim_time': use_sim_time_param,
        }.items(),
        condition=LaunchConfigurationEquals(
            launch_configuration_name='hardware_type',
            expected_value='fake'
        ),
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=robot_name_launch_arg,
        parameters=[
            {'robot_description': robot_description_launch_arg},
            ros2_control_controllers_config_parameter_file,
        ],
        output={'both': 'screen'},
    )

    spawn_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            '-c',
            f'/{robot_name_launch_arg.perform(context)}/controller_manager',
            'arm_controller',
        ],
        output={'both': 'screen'},
    )

    spawn_gripper_controller_node = Node(
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            '-c',
            f'/{robot_name_launch_arg.perform(context)}/controller_manager',
            'gripper_controller',
        ],
        output={'both': 'screen'},
    )

    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            '-c',
            f'/{robot_name_launch_arg.perform(context)}/controller_manager',
            'joint_state_broadcaster',
        ],
        condition=LaunchConfigurationEquals(
            launch_configuration_name='hardware_type',
            expected_value='fake'
        ),
        output={'both': 'screen'},
    )

    return [
        controller_manager_node,
        spawn_arm_controller_node,
        spawn_gripper_controller_node,
        spawn_joint_state_broadcaster_node,
        xsarm_control_launch_include,
        xsarm_descriptions_launch_include,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            choices=get_interbotix_xsarm_models(),
            description=(
                "model type of the Interbotix Arm such as 'wx200' or 'rx150'."
            ),
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
            default_value='false',
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_ros_control'),
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
            hardware_type='actual'
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
