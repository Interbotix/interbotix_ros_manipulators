from interbotix_ux_modules.arm import InterbotixManipulatorUX

# This script closes and opens the gripper twice, changing the gripper speed half way through
#
# To get started, open a terminal and type 'roslaunch interbotix_uxarm_control uxarm_control.launch robot_model:=uxarm6 use_gripper:=true robot_ip:=<robot_ip>'
# Then change to this directory and type 'python gripper_control.py'

def main():
    bot = InterbotixManipulatorUX("uxarm6", mode=0, wait_for_finish=True, gripper_type="gripper", ee_offset=[0, 0, 0.14, 0, 0, 0])
    bot.arm.go_to_holdup_pose()
    bot.gripper.move(pulse=450, delay=1.0)
    bot.gripper.open(delay=0.5)
    bot.gripper.config(pulse_vel=500)
    bot.gripper.close(delay=2.0)
    bot.gripper.open(delay=1.5)
    bot.arm.go_to_home_pose()

if __name__=='__main__':
    main()
