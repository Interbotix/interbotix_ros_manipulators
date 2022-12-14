# interbotix_xsarm_control

[![View Documentation](https://docs.trossenrobotics.com/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/arm_control.html)

## Overview

This package contains the configuration and launch files necessary to easily start the Interbotix Arm platform. This includes launching the **xs_sdk** node responsible for driving the Dynamixel motors on the robot and loading the URDF to the `robot_description` parameter. Essentially, this package is what all 'downstream' ROS packages should reference to get the robot up and running.

This package also contains simple scripts to demonstrate usage of the interface modules in the interbotix_ros_toolboxes repository. Currently, those interfaces are:

- [The Python-ROS2 API](demos/python_ros2_api/)
- [The MATLAB-ROS2 API](demos/matlab_ros2_api/)
