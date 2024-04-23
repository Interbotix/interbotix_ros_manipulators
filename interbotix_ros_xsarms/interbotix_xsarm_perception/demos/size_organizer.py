#!/usr/bin/env python3

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

import math
import time

from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

"""
This script uses a color/depth camera to get the arm to find different sized objects and sort them
by size. For this demo, the arm is placed to the right of the camera facing outward. When the
end-effector is located at x=0, y=0.3, z=0.2 w.r.t. the 'wx250s/base_link' frame, the AR tag should
be clearly visible to the camera. Objects will be placed in a single row on the right side of the
robot in descending size.

To get started, open a terminal and type:

    ros2 launch interbotix_xsarm_perception xsarm_perception.launch.py robot_model:=wx250s

Then change to this directory and type:

    python3 block_organizer.py
"""

ROBOT_MODEL = 'wx250s'
ROBOT_NAME = ROBOT_MODEL
REF_FRAME = 'camera_color_optical_frame'
ARM_TAG_FRAME = f'{ROBOT_NAME}/ar_tag_link'
ARM_BASE_FRAME = f'{ROBOT_NAME}/base_link'


def main():
    # Create a global node to serve as the backend for each API component
    global_node = create_interbotix_global_node()
    # Initialize the arm module along with the pointcloud and armtag modules
    bot = InterbotixManipulatorXS(
        robot_model=ROBOT_MODEL,
        robot_name=ROBOT_NAME,
        node=global_node,
    )
    pcl = InterbotixPointCloudInterface(
        node_inf=global_node
    )
    armtag = InterbotixArmTagInterface(
        ref_frame=REF_FRAME,
        arm_tag_frame=ARM_TAG_FRAME,
        arm_base_frame=ARM_BASE_FRAME,
        node_inf=global_node
    )

    robot_startup(global_node)

    # set the initial arm and gripper pose
    for joint_name in ['waist', 'shoulder', 'elbow']:
        bot.core.robot_set_motor_registers(
            cmd_type='single',
            name=joint_name,
            reg='Position_P_Gain',
            value=1500
        )
    bot.arm.go_to_sleep_pose()
    bot.gripper.release()

    # get the ArmTag pose
    time.sleep(0.5)
    armtag.find_ref_to_arm_base_transform()
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)

    # get the cluster positions
    # sort them from min to max 'y' position w.r.t. the ARM_BASE_FRAME
    success, clusters = pcl.get_cluster_positions(
        ref_frame=ARM_BASE_FRAME,
        sort_axis='y',
    )

    if success:
        # sort clusters by their size - from most points to min points
        sorted_clusters = sorted(clusters, key=lambda cluster: cluster['num_points'], reverse=True)

        # pick up all the objects and place in a single row based on size
        x_des = -0.07
        for cluster in sorted_clusters:
            x, y, z = cluster['position']
            z_dist = 0.2 - (z + 0.01)
            bot.arm.set_ee_pose_components(x=x, y=y, z=0.2, pitch=0.5)
            bot.arm.set_ee_cartesian_trajectory(z=-z_dist)
            print('about to close')
            bot.gripper.grasp()
            bot.arm.set_ee_cartesian_trajectory(z=z_dist)
            bot.arm.set_ee_pose_components(x=x_des, y=-0.3, z=0.2, pitch=0.5, yaw=-math.pi/2)
            bot.arm.set_ee_cartesian_trajectory(z=-z_dist)
            print('about to open')
            bot.gripper.release()
            bot.arm.set_ee_cartesian_trajectory(z=z_dist)
            x_des += 0.07
    else:
        print('Could not get cluster positions.')

    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.go_to_sleep_pose()
    robot_shutdown(global_node)


if __name__ == '__main__':
    main()
