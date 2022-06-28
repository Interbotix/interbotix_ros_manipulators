close all;

% This script commands currents [mA] to the arm joints
%
% To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=vx250'
% Then change to this directory and type 'python joint_current_control.py'

rosshutdown

joint_currents = [0, 200 , 200, 50, 0];
bot = InterbotixManipulatorXS("vx250", "arm", "gripper");
bot.dxl.robot_set_operating_modes("group", "arm", "current");
bot.dxl.robot_write_commands("arm", joint_currents);

rosshutdown
bot.stop_timers();
