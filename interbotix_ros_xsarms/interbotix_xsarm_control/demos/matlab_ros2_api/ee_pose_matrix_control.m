close all;

% This script makes the end-effector go to a specific pose only possible with a 6dof arm using a transformation matrix
%
% To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250s'
% Then change to this directory in your MATLAB console and type 'ee_pose_matrix_control'

rosshutdown

T_sd = eye(4);
T_sd(1,4) = 0.3;
T_sd(2,4) = 0;
T_sd(3,4) = 0.2;

bot = InterbotixManipulatorXS("wx250s", "arm", "gripper");
bot.arm.go_to_home_pose();
bot.arm.set_ee_pose_matrix(T_sd);
bot.arm.go_to_home_pose();
bot.arm.go_to_sleep_pose();

rosshutdown
bot.stop_timers();
