from interbotix_ux_modules.arm import InterbotixManipulatorUX

# This script commands some arbitrary positions to the arm joints
#
# To get started, open a terminal and type 'roslaunch interbotix_uxarm_control uxarm_control.launch robot_model:=uxarm6 use_gripper:=true robot_ip:=<robot_ip>'
# Then change to this directory and type 'python joint_position_control.py'

def main():
    bot = InterbotixManipulatorUX("uxarm6", robot_name="uxarm6", mode=0, gripper_type="gripper", ee_offset=[0, 0, 0.14, 0, 0, 0])
    joint_positions = [-1.0, -0.5, -0.5, 1.0, 0.7, -0.65]
    bot.arm.set_joint_positions(joint_positions, vel=0.35, accel=4.0, mode=0)
    bot.arm.go_to_holdup_pose()
    bot.arm.go_to_home_pose()

if __name__=='__main__':
    main()
