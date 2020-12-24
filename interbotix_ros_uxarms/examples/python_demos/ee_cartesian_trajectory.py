from interbotix_ux_modules.arm import InterbotixManipulatorUX

# This script makes the end-effector draw a square in 3D space; No gripper need be attached
#
# To get started, open a terminal and type 'roslaunch interbotix_uxarm_control uxarm_control.launch robot_model:=uxarm6 use_gripper:=false robot_ip:=<robot_ip>'
# Then change to this directory and type 'python ee_cartesian_trajectory.py'

def main():
    bot = InterbotixManipulatorUX("uxarm6", mode=0, wait_for_finish=True, gripper_type=None)
    bot.arm.go_to_holdup_pose()
    bot.arm.set_ee_cartesian_trajectory(z=0.2)
    bot.arm.set_ee_cartesian_trajectory(x=0.2)
    bot.arm.set_ee_cartesian_trajectory(z=-0.2)
    bot.arm.set_ee_cartesian_trajectory(x=-0.2)
    bot.arm.go_to_home_pose()

if __name__=='__main__':
    main()
