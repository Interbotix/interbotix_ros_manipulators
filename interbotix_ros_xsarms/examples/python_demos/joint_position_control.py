from interbotix_xs_modules.arm import InterbotixManipulatorXS

# This script commands some arbitrary positions to the arm joints
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250s'
# Then change to this directory and type 'python joint_position_control.py'

def main():
    joint_positions = [-1.0, 0.5 , 0.5, 0, -0.5, 1.57]
    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")
    bot.arm.go_to_home_pose()
    bot.arm.set_joint_positions(joint_positions)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
