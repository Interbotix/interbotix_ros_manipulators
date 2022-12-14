# interbotix_xsarm_joy

[![docs](https://docs.trossenrobotics.com/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/joystick_control.html)

## Overview

This package can be used to control the movements of any X-Series robotic arm using a SONY PS3/PS4 controller or Microsoft Xbox360 controller (untested) via Bluetooth. In this demo, the 'arm' joints are set to work in 'position' control mode while the gripper operates in 'PWM' mode. Refer to the joystick button map below to see how to operate the robot. Specifically, some of the joystick controls manipulate individual joints while others are used to perform 'inverse kinematics' on all the joints to get the end-effector of the robot (defined at 'ee_gripper_link') to move as if it's in Cartesian space. This is done using the [modern_robotics](https://github.com/NxRLab/ModernRobotics/tree/master/packages/Python) code library offered by Northwestern University.
