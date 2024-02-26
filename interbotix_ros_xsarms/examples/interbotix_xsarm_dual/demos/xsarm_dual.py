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
from threading import Thread

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import rclpy

"""
This script is used to make two WidowX-200 arms work in tandem with one another

To get started, open a terminal and type:

    ros2 launch interbotix_xsarm_dual xsarm_dual.launch.py

Then change to this directory and type:

    python3 xsarm_dual.py

Note that the 'robot_name' argument used when instantiating an InterbotixManipulatorXS instance
is the same name as the 'robot_name_X' launch file argument
"""


def robot_1():
    robot_1 = InterbotixManipulatorXS(
        robot_model='wx200',
        robot_name='arm_1',
        moving_time=1.0,
        gripper_pressure=1.0,
        start_on_init=False,
    )
    robot_1.arm.go_to_home_pose()
    robot_1.arm.set_ee_pose_components(x=0.3, z=0.2)
    robot_1.gripper.release(delay=0.05)
    robot_1.arm.set_single_joint_position('waist', math.pi/4.0)
    robot_1.gripper.grasp(delay=0.05)
    robot_1.arm.set_single_joint_position('waist', 0.0)
    robot_1.arm.go_to_sleep_pose()


def robot_2():
    robot_2 = InterbotixManipulatorXS(
        robot_model='wx200',
        robot_name='arm_2',
        moving_time=1.0,
        gripper_pressure=1.0,
        start_on_init=False,
    )
    robot_2.arm.go_to_home_pose()
    robot_2.arm.set_ee_pose_components(x=0.3, z=0.2)
    robot_2.gripper.release(delay=0.05)
    robot_2.arm.set_single_joint_position('waist', -math.pi/4.0)
    robot_2.gripper.grasp(delay=0.05)
    robot_2.arm.set_single_joint_position('waist', 0.0)
    robot_2.arm.go_to_sleep_pose()


def main():
    rclpy.init()
    Thread(target=robot_1).start()
    Thread(target=robot_2).start()


if __name__ == '__main__':
    main()
