# Copyright 2023 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def generate_launch_description():
    # PATHを追加で通さないとSTLファイルが読み込まれない
    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
           'IGN_GAZEBO_RESOURCE_PATH': os.path.dirname(
               get_package_share_directory('interbotix_xsarm_descriptions'))}
    # -r オプションで起動時にシミュレーションをスタートしないと、コントローラが起動しない
    ign_gazebo = ExecuteProcess(
            cmd=['ign gazebo -r empty.sdf'],
            output='screen',
            additional_env=env,
            shell=True
        )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/mobile_wx200/robot_description',
                   '-name', 'mobile_wx200',
                   '-allow_renaming', 'true'],
    )



    bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
                output='screen'
            )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        ign_gazebo,
        ignition_spawn_entity,
        bridge
    ])
