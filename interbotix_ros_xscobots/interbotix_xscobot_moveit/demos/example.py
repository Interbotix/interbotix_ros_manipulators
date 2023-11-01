#!/usr/bin/env python3

# Copyright 2023 Trossen Robotics
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

from interbotix_moveit_interface import moveit_interface_core
from interbotix_moveit_interface import move_arm

import numpy as np

"""
This script is working demonstration of using the interbotix_moveit_interface
to program the manipulator easily using the developed python api. This example
is designed for the `dx400` xscobot hardware. Make sure to adjust andchange the
passed hard-coded values as per the application.

Note: Please make sure to initialize the moveit_py node using the `moveit_api.launch.py`

To run this example, use the following command in terminal:

    ros2 launch interbotix_xscobot_moveit moveit_api.launch.py robot_model:=dx400

"""


def main():
    bot = move_arm.InterbotixManipulatorXS(
        robot_model="dx400",
        group_name="interbotix_arm",
        gripper_name="interbotix_gripper",
    )

    joint_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    bot.arm.go_to_joint_positions(joint_pos)
    bot.arm.go_to_sleep_pose()
    bot.gripper.gripper_open()
    bot.gripper.gripper_close()


if __name__ == "__main__":
    main()