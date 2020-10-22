from interbotix_xs_modules.arm import InterbotixManipulatorXS

# This script makes the end-effector draw a square in 3D space
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250'
# Then change to this directory and type 'python ee_cartesian_trajectory.py'

def main():
    bot = InterbotixManipulatorXS("wx250", "arm", "gripper")
    bot.arm.go_to_home_pose()
    bot.arm.set_ee_cartesian_trajectory(z=-0.2)
    bot.arm.set_ee_cartesian_trajectory(x=-0.2)
    bot.arm.set_ee_cartesian_trajectory(z=0.2)
    bot.arm.set_ee_cartesian_trajectory(x=0.2)
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
