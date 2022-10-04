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
    construct_interbotix_xsarm_semantic_robot_description_command,
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
from interbotix_xs_modules.xs_launch.xs_launch import determine_use_sim_time_param
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
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    base_link_frame_launch_arg = LaunchConfiguration('base_link_frame')
    show_ar_tag_launch_arg = LaunchConfiguration('show_ar_tag')
    use_world_frame_launch_arg = LaunchConfiguration('use_world_frame')
    external_urdf_loc_launch_arg = LaunchConfiguration('external_urdf_loc')
    external_srdf_loc_launch_arg = LaunchConfiguration('external_srdf_loc')
    mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    use_moveit_rviz_launch_arg = LaunchConfiguration('use_moveit_rviz')
    rviz_frame_launch_arg = LaunchConfiguration('rviz_frame')
    rviz_config_file_launch_arg = LaunchConfiguration('rviz_config_file')
    world_filepath_launch_arg = LaunchConfiguration('world_filepath')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    use_moveit_interface_gui_launch_arg = LaunchConfiguration('use_moveit_interface_gui')

    # sets use_sim_time parameter to 'true' if using gazebo hardware
    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        hardware_type_launch_arg=hardware_type_launch_arg
    )
    config_path = PathJoinSubstitution([
        FindPackageShare('interbotix_xsarm_moveit'),
        'config',
    ])

    robot_description_semantic = construct_interbotix_xsarm_semantic_robot_description_command(
        robot_model=robot_model_launch_arg.perform(context),
        config_path=config_path,
    )

    moveit_interface_node = Node(
        package='interbotix_moveit_interface',
        executable='moveit_interface',
        # namespace=robot_name_launch_arg,
        condition=LaunchConfigurationEquals(
            'moveit_interface_type',
            expected_value='cpp'
        ),
        parameters=[{
            'robot_description_semantic': robot_description_semantic,
            'use_sim_time': use_sim_time_param,
        }],
        remappings=(
            ('/joint_states', f'/{robot_name_launch_arg.perform(context)}/joint_states'),
            ('/robot_description', f'/{robot_name_launch_arg.perform(context)}/robot_description'),
        )
    )

    moveit_interface_gui_node = Node(
        package='interbotix_moveit_interface',
        executable='moveit_interface_gui',
        # namespace=robot_name_launch_arg,
        condition=(
            LaunchConfigurationEquals(
                launch_configuration_name='moveit_interface_type',
                expected_value='cpp'
            ) and IfCondition(
                use_moveit_interface_gui_launch_arg
            )
        ),
        parameters=[{
            'use_sim_time': use_sim_time_param,
        }],
    )

    xsarm_moveit_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit'),
                'launch',
                'xsarm_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch_arg,
            'base_link_frame': base_link_frame_launch_arg,
            'show_ar_tag': show_ar_tag_launch_arg,
            'use_world_frame': use_world_frame_launch_arg,
            'external_urdf_loc': external_urdf_loc_launch_arg,
            'external_srdf_loc': external_srdf_loc_launch_arg,
            'mode_configs': mode_configs_launch_arg,
            'use_moveit_rviz': use_moveit_rviz_launch_arg,
            'rviz_frame': rviz_frame_launch_arg,
            'rviz_config_file': rviz_config_file_launch_arg,
            'hardware_type': hardware_type_launch_arg,
            'world_filepath': world_filepath_launch_arg,
            'robot_description': robot_description_launch_arg,
            'use_sim_time': use_sim_time_param,
        }.items(),
    )

    return [
        moveit_interface_node,
        moveit_interface_gui_node,
        xsarm_moveit_launch_include,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            choices=get_interbotix_xsarm_models(),
            description="model type of the Interbotix Arm such as 'wx200' or 'rx150'.",
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
                'the file path to the custom semantic description file that you would '
                "like to include in the Interbotix robot's semantic description."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit_interface'),
                'config',
                'modes.yaml',
            ]),
            description="the file path to the 'mode config' YAML file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_moveit_rviz',
            default_value='true',
            choices=('true', 'false'),
            description="launches RViz with MoveIt's RViz configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_frame',
            default_value='world',
            description=(
                'defines the fixed frame parameter in RViz. Note that if '
                '`use_world_frame` is `false`, this parameter should be changed to a frame'
                ' that exists.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit_interface'),
                'rviz',
                'xsarm_moveit_interface.rviz'
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
            'moveit_interface_type',
            default_value='cpp',
            choices=(
                'cpp',
                # 'python',
            ),
            description=(
                "if 'cpp', launches the custom moveit_interface C++ API node; if 'python', launch "
                'the Python Interface tutorial node; only the cpp option is currently supported.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_moveit_interface_gui',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'launch a custom GUI to interface with the moveit_interface node so that the user '
                "can command specific end-effector poses (defined by 'ee_gripper_link')."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                "published over the ROS topic /clock; this value is automatically set to 'true' if"
                ' using Gazebo hardware.'
            )
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            show_gripper_bar='true',
            show_gripper_fingers='true',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
