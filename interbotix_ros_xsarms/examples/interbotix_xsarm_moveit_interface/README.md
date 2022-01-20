# interbotix_xsarm_moveit_interface

[![docs](https://trossenrobotics.com/docs/docs_button.svg)](https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_packages/moveit_interface_and_api.html)

## Overview

This package contains a small API modeled after the [Move Group C++ Interface Tutorial](https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_interface/src/move_group_interface_tutorial.cpp) that allows a user to command desired end-effector poses to an Interbotix arm. It is not meant to be all-encompassing but rather should be viewed as a starting point for someone interested in creating their own MoveIt interface to interact with an arm. The package also contains a small GUI that can be used to pose the end-effector.

Finally, this package also contains a modified version of the [Move Group Python Interface Tutorial](https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py) script that can be used as a guide for those users who would like to interface with an Interbotix robot via the MoveIt Commander Python module.
