#!/bin/bash

# This script is called by the 'xsarm_rpi4_boot.service' file when
# the Raspberry Pi boots. It just sources the ROS related workspaces
# and launches the xsarm_joy launch file.

source /opt/ros/melodic/setup.bash
source /home/pibot/interbotix_ws/devel/setup.bash
roslaunch interbotix_xsarm_joy xsarm_joy.launch use_rviz:=false robot_model:=$ROBOT_MODEL
