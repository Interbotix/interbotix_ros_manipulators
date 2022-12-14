# interbotix_xsarm_diagnostic_tool

## Overview

This package is meant to be used as a way to analyze joint data over time. For example, if the user would like the arm to perform a certain task but is not sure if it will 'strain' a specific arm joint too much, then this tool would be an easy way to record, save (to a rosbag and CSV file), and view data live while the joint in question rotates.

The package provides two methods for doing this task: the Diagnostic Tool and the Diagnostic Listener. The package also provides a helper script to convert bag file data to a CSV format.

### Diagnostic Tool

[![docs](https://docs.trossenrobotics.com/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros1_packages/arm_diagnostic_tool.html)

The diagnostic tool cycles a specified joint through a sinusoidal trajectory for a specified time. While the tool is running, it continuously publishes the joint position, velocity, effort, temperature, and goal state. The launch file also launches bag recording to track these topics, and rqt_plot windows to visualize them.

### Diagnostic Listener

[![docs](https://docs.trossenrobotics.com/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros1_packages/arm_diagnostic_listener.html)

The diagnostic listener is intended to get joint information like present temperature from the robot while it executes any commanded trajectory. This is useful for when you would like the robot to move through a more complex motion than the more basic Diagnostic Tool is able to provide. The Listener subscribes to the joint states of the [configured](config/listener.yaml) joints, periodically retrieves the Present_Temperature register from the DYNAMIXEL servos, and publishes the data to the namespaced `temperatures/joint_group` topic.
