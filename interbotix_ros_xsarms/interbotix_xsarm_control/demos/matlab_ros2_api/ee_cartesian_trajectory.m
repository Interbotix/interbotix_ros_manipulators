close all;

% This script makes the end-effector draw a square in 3D space
%
% To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250'
% Then change to this directory in your MATLAB console and type 'ee_cartesian_trajectory'

rosshutdown

bot = InterbotixManipulatorXS("wx250", "arm", "gripper");
bot.arm.go_to_home_pose();
bot.arm.set_ee_cartesian_trajectory(z=-0.2);
bot.arm.set_ee_cartesian_trajectory(x=-0.2);
bot.arm.set_ee_cartesian_trajectory(z=0.2);
bot.arm.set_ee_cartesian_trajectory(x=0.2);
bot.arm.go_to_sleep_pose();

rosshutdown
bot.stop_timers();
