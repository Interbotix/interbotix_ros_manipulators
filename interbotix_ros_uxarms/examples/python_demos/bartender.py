from interbotix_ux_modules.arm import InterbotixManipulatorUX
import math

# This script makes the end-effector perform pick, pour, and place tasks
#
# To get started, open a terminal and type 'roslaunch interbotix_uxarm_control uxarm_control.launch robot_model:=uxarm6 use_gripper:=true robot_ip:=<robot_ip>'
# Then change to this directory and type 'python bartender.py'

def main():
    bot = InterbotixManipulatorUX("uxarm6", mode=0, wait_for_finish=True, gripper_type="gripper", ee_offset=[0, 0, 0.14, 0, 0, 0])
    bot.arm.go_to_holdup_pose()
    bot.arm.set_single_joint_position("joint1", math.pi/2.0)
    bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.05)
    bot.gripper.move(450)
    bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.05)
    bot.arm.set_single_joint_position("joint1", -math.pi/2.0)
    bot.arm.set_ee_cartesian_trajectory(pitch=1.57)
    bot.arm.set_ee_cartesian_trajectory(pitch=-1.57)
    bot.arm.set_single_joint_position("joint1", math.pi/2.0)
    bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.05)
    bot.gripper.open()
    bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.05)
    bot.arm.go_to_holdup_pose()
    bot.arm.go_to_home_pose()

if __name__=='__main__':
    main()
