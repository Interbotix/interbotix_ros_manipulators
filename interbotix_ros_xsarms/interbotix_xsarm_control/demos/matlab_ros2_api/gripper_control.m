close all;

% This script closes and opens the gripper twice, changing the gripper pressure half way through
%
% To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx200'
% Then change to this directory in your MATLAB console and type 'gripper_control'

rosshutdown

arm = InterbotixManipulatorXS("wx200", "arm", "gripper");
arm.gripper.close(2.0);
arm.gripper.open(2.0);
arm.gripper.set_pressure(1.0);
arm.gripper.close(2.0);
arm.gripper.open(2.0);

rosshutdown
bot.stop_timers();
