from interbotix_xs_modules.arm import InterbotixManipulatorXS

# This script commands currents [mA] to the arm joints
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=vx250'
# Then change to this directory and type 'python joint_current_control.py'

def main():
    joint_pwms = [0, 100 , 100, 25, 0]
    bot = InterbotixManipulatorXS("vx250", "arm", "gripper")
    bot.dxl.robot_set_operating_modes("group", "arm", "pwm")
    bot.dxl.robot_write_commands("arm", joint_pwms)

if __name__=='__main__':
    main()
