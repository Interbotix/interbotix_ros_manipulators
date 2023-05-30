from interbotix_xs_modules.arm import InterbotixManipulatorXS
import sys

# This script makes the end-effector draw a square in 3D space
# Note that this script may not work for every arm as it was designed for the wx250
# Make sure to adjust commanded joint positions and poses as necessary
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250'
# Then change to this directory and type 'python ee_cartesian_trajectory.py  # python3 bartender.py if using ROS Noetic'

def main():
    bot = InterbotixManipulatorXS("wx250", "arm", "gripper")

    if (bot.arm.group_info.num_joints < 5):
        print('This demo requires the robot to have at least 5 joints!')
        sys.exit()

    bot.arm.go_to_home_pose()
    bot.arm.set_ee_cartesian_trajectory(z=-0.2)
    bot.arm.set_ee_cartesian_trajectory(x=-0.2)
    bot.arm.set_ee_cartesian_trajectory(z=0.2)
    bot.arm.set_ee_cartesian_trajectory(x=0.2)
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
