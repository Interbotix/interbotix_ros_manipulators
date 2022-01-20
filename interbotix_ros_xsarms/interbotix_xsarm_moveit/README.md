# interbotix_xsarm_moveit

[![docs](https://trossenrobotics.com/docs/docs_button.svg)](https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_packages/moveit_motion_planning_configuration.html)

## Overview

This package contains the necessary config files to get any of the many Interbotix X-Series arms working with MoveIt. Originally, the MoveIt Setup Assistant wizard was used to generate a MoveIt package for each robot individually. The packages were then all merged into one and the launch files modified so that specific arguments (like `robot_model`, `dof`, and `robot_name`) could be passed down to load the right config files (such as the SRDFs). Additionally, this package makes use of the FollowJointTrajectory interface which seems to work pretty well in both Gazebo and on the physical robot. A 'master' launch file was then written to allow a user to choose whether to have MoveIt work with the simulated version, the physical robot hardware, or a MoveIt generated fake robot.
