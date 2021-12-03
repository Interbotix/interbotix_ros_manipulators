close all;

% This script makes the end-effector go to a specific pose by defining the pose components
%
% To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250'
% Then change to this directory in your MATLAB console and type 'ee_pose_components'

rosshutdown

bot = InterbotixManipulatorXS("wx250", "arm", "gripper");
bot.arm.go_to_home_pose();
bot.arm.set_ee_pose_components(x=0.2, y=0.1, z=0.2, roll=1.0, pitch=1.5);
bot.arm.go_to_home_pose();
bot.arm.go_to_sleep_pose();

rosshutdown
bot.stop_timers();
