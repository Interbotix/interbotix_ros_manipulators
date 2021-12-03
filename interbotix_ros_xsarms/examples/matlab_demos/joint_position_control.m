close all;

% This script commands some arbitrary positions to the arm joints
%
% To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250s'
% Then change to this directory in your MATLAB console and type 'joint_position_control'

rosshutdown

joint_positions = [-1.0, 0.5 , 0.5, 0, -0.5, 1.57];
bot = InterbotixManipulatorXS("wx250s", "arm", "gripper");
bot.arm.go_to_home_pose();
bot.arm.set_joint_positions(joint_positions);
bot.arm.go_to_home_pose();
bot.arm.go_to_sleep_pose();

rosshutdown
bot.stop_timers();
