from interbotix_ux_modules.arm import InterbotixManipulatorUX
from interbotix_common_modules import angle_manipulation as ang
import numpy as np

# This script makes the end-effector go to a specific pose only possible with a 6dof arm using a transformation matrix; No gripper need be attached
#
# To get started, open a terminal and type 'roslaunch interbotix_uxarm_control uxarm_control.launch robot_model:=uxarm6 use_gripper:=false robot_ip:=<robot_ip>'
# Then change to this directory and type 'python ee_pose_matrix.py'

def main():
    T_sd = np.identity(4)
    T_sd[0,3] = 0.3
    T_sd[1,3] = -0.1
    T_sd[2,3] = 0.2
    rpy = [3.14, -1.2, 0.7]
    T_sd[:3,:3] = ang.eulerAnglesToRotationMatrix(rpy)
    bot = InterbotixManipulatorUX("uxarm6", mode=0, wait_for_finish=True, gripper_type=None)
    bot.arm.go_to_holdup_pose()
    bot.arm.set_ee_pose_matrix(T_sd)
    bot.arm.go_to_home_pose()

if __name__=='__main__':
    main()
