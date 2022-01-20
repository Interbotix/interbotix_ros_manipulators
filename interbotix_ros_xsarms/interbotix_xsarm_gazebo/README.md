# interbotix_xsarm_gazebo

[![docs](https://trossenrobotics.com/docs/docs_button.svg)](https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_packages/gazebo_simulation_configuration.html)

## Overview

This package contains the necessary config files to get any of the many Interbotix X-Series arms simulated in Gazebo. Specifically, it contains the [interbotix_texture.gazebo](config/interbotix_texture.gazebo) file which allows the black texture of the robotic arms to display properly (following the method explained [here](http://answers.gazebosim.org/question/16280/how-to-use-custom-textures-on-urdf-models-in-gazebo/)). It also contains YAML files with tuned PID gains for the arm and gripper joints so that ros_control can control the arms effectively. This package has one of two applications. It can either be used in conjunction with MoveIt via the FollowJointTrajectory interface or it can be used by itself via the JointPositionController interface.
