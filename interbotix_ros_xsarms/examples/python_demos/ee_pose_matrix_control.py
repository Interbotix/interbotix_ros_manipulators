from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np

# This script makes the end-effector go to a specific pose only possible with a 6dof arm using a transformation matrix
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250s'
# Then change to this directory and type 'python ee_pose_matrix_control.py'

def main():
    T_sd = np.identity(4)
    T_sd[0,3] = 0.3
    T_sd[1,3] = 0
    T_sd[2,3] = 0.2

    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")
    bot.arm.go_to_home_pose()
    bot.arm.set_ee_pose_matrix(T_sd)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
