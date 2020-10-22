from interbotix_xs_modules.arm import InterbotixManipulatorXS
# This script closes and opens the gripper twice, changing the gripper pressure half way through
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx200'
# Then change to this directory and type 'python gripper_control.py'

def main():
    arm = InterbotixManipulatorXS("wx200", "arm", "gripper")
    arm.gripper.close(2.0)
    arm.gripper.open(2.0)
    arm.gripper.set_pressure(1.0)
    arm.gripper.close(2.0)
    arm.gripper.open(2.0)

if __name__=='__main__':
    main()
