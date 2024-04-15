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

from colorsys import rgb_to_hsv
from enum import Enum
from typing import List

from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

"""
This script uses a color/depth camera to get the arm to find blocks and sort them by color. For
this demo, the arm is placed to the right of the camera facing outward. When the end-effector is
located at x=0, y=0.3, z=0.2 w.r.t. the 'wx200/base_link' frame, the AR tag should be clearly
visible to the camera. Four small baskets should also be placed in front and to the right of the
arm.

To get started, open a terminal and type:

    ros2 launch interbotix_xsarm_perception xsarm_perception.launch.py robot_model:=wx200

Then change to this directory and type:

    python3 color_sorter.py
"""


class Color(Enum):
    UNKNOWN = 0
    RED = 1
    ORANGE = 2
    YELLOW = 3
    GREEN = 4
    BLUE = 5
    PURPLE = 6


ROBOT_MODEL = 'wx200'
ROBOT_NAME = ROBOT_MODEL
REF_FRAME = 'camera_color_optical_frame'
ARM_TAG_FRAME = f'{ROBOT_NAME}/ar_tag_link'
ARM_BASE_FRAME = f'{ROBOT_NAME}/base_link'


def main():
    # Create a global node to serve as the backend for each API component
    global_node = create_interbotix_global_node()
    # Initialize the arm, pointcloud, and armtag modules
    bot = InterbotixManipulatorXS(
        robot_model=ROBOT_MODEL,
        robot_name=ROBOT_NAME,
        node=global_node,
    )
    pcl = InterbotixPointCloudInterface(
        node_inf=global_node,
    )
    armtag = InterbotixArmTagInterface(
        ref_frame=REF_FRAME,
        arm_tag_frame=ARM_TAG_FRAME,
        arm_base_frame=ARM_BASE_FRAME,
        node_inf=global_node,
    )

    # Start up the API
    robot_startup(global_node)

    # set initial arm and gripper pose
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
    armtag.find_ref_to_arm_base_transform()
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)

    # get the cluster positions
    # sort them from max to min 'x' position w.r.t. the ARM_BASE_FRAME
    success, clusters = pcl.get_cluster_positions(
        ref_frame=ARM_BASE_FRAME,
        sort_axis='y',
        reverse=True
    )

    if success:
        # pick up all the objects and drop them in baskets
        for cluster in clusters:
            x, y, z = cluster['position']
            print(x, y, z)
            bot.arm.set_ee_pose_components(x=x, y=y, z=0.1, pitch=0.5)
            bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.02, pitch=0.5)
            bot.gripper.grasp()
            bot.arm.set_ee_pose_components(x=x, y=y, z=0.1, pitch=0.5)
            clr = color_compare(cluster['color'])
            if (clr == Color.RED):
                bot.arm.set_ee_pose_components(x=0.24, y=-0.1, z=0.2)
            elif (clr == Color.YELLOW):
                bot.arm.set_ee_pose_components(x=0.38, y=-0.24, z=0.2)
            elif (clr == Color.PURPLE):
                bot.arm.set_ee_pose_components(x=0.24, y=-0.24, z=0.2)
            elif (clr == Color.BLUE):
                bot.arm.set_ee_pose_components(x=0.38, y=-0.1, z=0.2)
            else:
                # if color cannot be recognized, then put the block back...
                bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.01, pitch=0.5)
            bot.gripper.release()
    else:
        print('Could not get cluster positions.')

    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.go_to_sleep_pose()
    robot_shutdown(global_node)


def color_compare(rgb: List[float]) -> Color:
    """
    Determine the color of each object using the Hue value in the HSV color space.

    :param rgb: List of 3 floats describing the color in [R, G, B]
    :return: A Color
    """
    r, g, b = [x/255.0 for x in rgb]
    h, _, _ = rgb_to_hsv(r, g, b)

    if h < 0.025:
        return Color.RED
    elif 0.025 < h < 0.05:
        return Color.ORANGE
    elif 0.1 < h < 0.15:
        return Color.YELLOW
    elif 0.3 < h < 0.4:
        return Color.GREEN
    elif 0.55 < h < 0.65:
        return Color.BLUE
    elif 0.75 < h < 0.85:
        return Color.PURPLE
    else:
        return Color.UNKNOWN


if __name__ == '__main__':
    main()
