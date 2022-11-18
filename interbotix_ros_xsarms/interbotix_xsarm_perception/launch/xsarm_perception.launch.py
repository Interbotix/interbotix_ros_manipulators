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

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    load_configs_launch_arg = LaunchConfiguration('load_configs')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')

    pointcloud_enable_launch_arg = LaunchConfiguration('rs_camera_pointcloud_enable')
    rbg_camera_profile_launch_arg = LaunchConfiguration('rs_camera_rbg_camera_profile')
    depth_module_profile_launch_arg = LaunchConfiguration('rs_camera_depth_module_profile')
    logging_level_launch_arg = LaunchConfiguration('rs_camera_logging_level')
    output_location_launch_arg = LaunchConfiguration('rs_camera_output_location')
    initial_reset_launch_arg = LaunchConfiguration('rs_camera_initial_reset')

    filter_ns_launch_arg = LaunchConfiguration('filter_ns')
    filter_params_launch_arg = LaunchConfiguration('filter_params')
    use_pointcloud_tuner_gui_launch_arg = LaunchConfiguration('use_pointcloud_tuner_gui')
    enable_pipeline_launch_arg = LaunchConfiguration('enable_pipeline')
    cloud_topic_launch_arg = LaunchConfiguration('cloud_topic')

    tags_config_launch_arg = LaunchConfiguration('tags_config')
    camera_frame_launch_arg = LaunchConfiguration('camera_frame')
    apriltag_ns_launch_arg = LaunchConfiguration('apriltag_ns')
    camera_color_topic_launch_arg = LaunchConfiguration('camera_color_topic')
    camera_info_topic_launch_arg = LaunchConfiguration('camera_info_topic')
    armtag_ns_launch_arg = LaunchConfiguration('armtag_ns')
    ref_frame_launch_arg = LaunchConfiguration('ref_frame')
    arm_base_frame_launch_arg = LaunchConfiguration('arm_base_frame')
    arm_tag_frame_launch_arg = LaunchConfiguration('arm_tag_frame')
    use_armtag_tuner_gui_launch_arg = LaunchConfiguration('use_armtag_tuner_gui')
    position_only_launch_arg = LaunchConfiguration('position_only')

    load_transforms_launch_arg = LaunchConfiguration('load_transforms')
    transform_filepath_launch_arg = LaunchConfiguration('transform_filepath')

    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    rviz_frame_launch_arg = LaunchConfiguration('rviz_frame')
    rvizconfig_launch_arg = LaunchConfiguration('rvizconfig')

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
            'load_configs': load_configs_launch_arg,
            'robot_description': robot_description_launch_arg,
            'use_rviz': 'false',
            'hardware_type': 'actual',
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }.items()
    )

    rs_camera_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ]),
        launch_arguments={
            'camera_name': 'camera',
            'rgb_camera.profile': rbg_camera_profile_launch_arg,
            'depth_module.profile': depth_module_profile_launch_arg,
            'pointcloud.enable': pointcloud_enable_launch_arg,
            'initial_reset': initial_reset_launch_arg,
            'log_level': logging_level_launch_arg,
            'output': output_location_launch_arg,
        }.items()
    )

    pc_filter_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_perception_modules'),
                'launch',
                'pc_filter.launch.py',
            ])
        ]),
        launch_arguments={
            'filter_ns': filter_ns_launch_arg,
            'filter_params': filter_params_launch_arg,
            'enable_pipeline': enable_pipeline_launch_arg,
            'cloud_topic': cloud_topic_launch_arg,
            'use_pointcloud_tuner_gui': use_pointcloud_tuner_gui_launch_arg,
        }.items(),
    )

    armtag_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_perception_modules'),
                'launch',
                'armtag.launch.py',
            ])
        ]),
        launch_arguments={
            'tags_config': tags_config_launch_arg,
            'camera_frame': camera_frame_launch_arg,
            'apriltag_ns': apriltag_ns_launch_arg,
            'camera_color_topic': camera_color_topic_launch_arg,
            'camera_info_topic': camera_info_topic_launch_arg,
            'armtag_ns': armtag_ns_launch_arg,
            'ref_frame': ref_frame_launch_arg,
            'arm_base_frame': arm_base_frame_launch_arg,
            'arm_tag_frame': arm_tag_frame_launch_arg,
            'use_armtag_tuner_gui': use_armtag_tuner_gui_launch_arg,
            'position_only': position_only_launch_arg,
        }.items()
    )

    static_transform_pub_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_tf_tools'),
                'launch',
                'static_transform_pub.launch.py',
            ])
        ]),
        launch_arguments={
            'load_transforms': load_transforms_launch_arg,
            'transform_filepath': transform_filepath_launch_arg,
        }.items()
    )

    xsarm_perception_rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=robot_name_launch_arg,
        arguments=[
            '-f', rviz_frame_launch_arg,
            '-d', rvizconfig_launch_arg,
        ],
        output={'both': 'log'},
        condition=IfCondition(use_rviz_launch_arg.perform(context)),
    )

    return [
        xsarm_control_launch_include,
        rs_camera_launch_include,
        pc_filter_launch_include,
        armtag_launch_include,
        static_transform_pub_launch_include,
        xsarm_perception_rviz2_node,
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
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            show_ar_tag='true',
            use_world_frame='false'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'load_configs',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'a boolean that specifies whether or not the initial register values (under the '
                "'motors' heading) in a Motor Config file should be written to the motors; as the "
                "values being written are stored in each motor's EEPROM (which means the values "
                'are retained even after a power cycle), this can be set to false after the first '
                'time using the robot. Setting to false also shortens the node startup time by a '
                'few seconds and preserves the life of the EEPROM.'
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
            'rs_camera_pointcloud_enable',
            default_value='true',
            choices=('true', 'false'),
            description="enables the RealSense camera's pointcloud.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_rbg_camera_profile',
            default_value='640x480x30',
            description='profile for the rbg camera image stream, in `<width>x<height>x<fps>`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_depth_module_profile',
            default_value='640x480x30',
            description='profile for the depth module stream, in `<width>x<height>x<fps>`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_logging_level',
            default_value='info',
            choices=('debug', 'info', 'warn', 'error', 'fatal'),
            description='set the logging level for the realsense2_camera launch include.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_output_location',
            default_value='screen',
            choices=('screen', 'log'),
            description='set the logging location for the realsense2_camera launch include.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_initial_reset',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'On occasions the RealSense camera is not closed properly and due to firmware '
                'issues needs to reset. If set to `true`, the device will reset prior to usage.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'filter_ns',
            default_value='pc_filter',
            description='namespace where the pointcloud related nodes and parameters are located.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'filter_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_perception'),
                'config',
                'filter_params.yaml'
            ]),
            description=(
                'file location of the parameters used to tune the perception pipeline filters.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_pointcloud_tuner_gui',
            default_value='false',
            choices=('true', 'false'),
            description='whether to show a GUI that a user can use to tune filter parameters.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'enable_pipeline',
            default_value=LaunchConfiguration('use_pointcloud_tuner_gui'),
            choices=('true', 'false'),
            description=(
                'whether to enable the perception pipeline filters to run continuously; to save '
                'computer processing power, this should be set to `false` unless you are actively '
                'trying to tune the filter parameters; if `false`, the pipeline will only run if '
                'the `get_cluster_positions` ROS service is called.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/camera/depth/color/points',
            description='the absolute ROS topic name to subscribe to raw pointcloud data.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'tags_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_perception_modules'),
                'config',
                'tags.yaml'
            ]),
            description='parameter file location for the AprilTag configuration.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera_color_optical_frame',
            description='the camera frame in which the AprilTag will be detected.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'apriltag_ns',
            default_value='apriltag',
            description='namespace where the AprilTag related nodes and parameters are located.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_color_topic',
            default_value='camera/color/image_raw',
            description='the absolute ROS topic name to subscribe to color images.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='camera/color/camera_info',
            description='the absolute ROS topic name to subscribe to the camera color info.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'armtag_ns',
            default_value='armtag',
            description='name-space where the Armtag related nodes and parameters are located.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ref_frame',
            default_value=LaunchConfiguration('camera_frame'),
            description=(
                'the reference frame that the armtag node should use when publishing a static '
                'transform for where the arm is relative to the camera,'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_base_frame',
            default_value=[
                LaunchConfiguration('robot_name'), '/', LaunchConfiguration('base_link_frame')
            ],
            description=(
                'the child frame that the armtag node should use when publishing a static '
                'transform for where the arm is relative to the camera.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_tag_frame',
            default_value=[
                LaunchConfiguration('robot_name'), '/', 'ar_tag_link'
            ],
            description=(
                'name of the frame on the arm where the AprilTag is located (typically defined in '
                'the URDF).'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_armtag_tuner_gui',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'whether to show a GUI that a user can use to publish the `ref_frame` to '
                '`arm_base_frame` transform.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'position_only',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'whether only the position component of the detected AprilTag pose should be used '
                'when calculating the `ref_frame` to `arm_base_frame` transform; this should only '
                'be set to `true` if a TF chain already exists connecting the camera and arm '
                'base_link frame, and you just want to use the AprilTag to refine the pose '
                'further.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'load_transforms',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'whether or not the static_trans_pub node should publish any poses stored in the '
                'static_transforms.yaml file at startup; this should only be set to `false` if a '
                'TF chain already exists connecting the camera and arm base_link frame (typically '
                'defined in a URDF), and you would rather use that TF chain as opposed to the one '
                'specified in the static_transforms.yaml file.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'transform_filepath',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_perception'),
                'config',
                'static_transforms.yaml'
            ]),
            description=(
                'filepath to the static_transforms.yaml file used by the static_trans_pub node; if'
                ' the file does not exist yet, this is where you would like the file to be '
                'generated.'
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
            'rviz_frame',
            default_value=[
                    LaunchConfiguration('robot_name'),
                    '/',
                    LaunchConfiguration('base_link_frame')
            ],
            description='desired `fixed frame` in RViz.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_perception'),
                'rviz',
                'xsarm_perception.rviz'
            ]),
            description='filepath to the RViz config file.',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
