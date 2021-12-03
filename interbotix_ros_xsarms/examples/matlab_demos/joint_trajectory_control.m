close all;

% This script commands an arbitrary trajectory to the arm joints
%
% To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250s'
% Then change to this directory in your MATLAB console and type 'joint_trajectory_control'

rosshutdown

trajectory_times = {0.0, 2.0, 4.0, 6.0};
trajectory_points = {
    [0.0,  0.0, 0.0, 0.0, 0.0, 0.0]
    [0.0,  0.0, 0.0, 0.0, 0.5, 0.0]
    [0.5,  0.0, 0.0, 0.0, 0.5, 0.0]
    [-0.5, 0.0, 0.0, 0.0, 0.5, 0.0]};
trajectory = containers.Map(trajectory_times, trajectory_points);

bot = InterbotixManipulatorXS("wx250s", "arm", "gripper");
bot.arm.go_to_home_pose()
bot.dxl.robot_write_trajectory("group", "arm", "position", trajectory)
bot.arm.go_to_home_pose()
bot.arm.go_to_sleep_pose()

rosshutdown
bot.stop_timers();
