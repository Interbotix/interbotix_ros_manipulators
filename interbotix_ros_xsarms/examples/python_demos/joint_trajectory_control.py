from interbotix_xs_modules.arm import InterbotixManipulatorXS
from time import sleep

# This script commands an arbitrary trajectory to the arm joints
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250s'
# Then change to this directory and type 'python joint_trajectory_control  # python3 bartender.py if using ROS Noetic'

def main():

    trajectory = [
        {0.0: [0.0,  0.0, 0.0, 0.0, 0.0, 0.0]},
        {2.0: [0.0,  0.0, 0.0, 0.0, 0.5, 0.0]},
        {4.0: [0.5,  0.0, 0.0, 0.0, 0.5, 0.0]},
        {6.0: [-0.5, 0.0, 0.0, 0.0, 0.5, 0.0]}
    ]

    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")
    bot.arm.go_to_home_pose()
    bot.dxl.robot_write_trajectory("group", "arm", "position", trajectory)
    sleep(6.0)  # sleep to ensure trajectory has time to complete
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

if __name__ == '__main__':
    main()
