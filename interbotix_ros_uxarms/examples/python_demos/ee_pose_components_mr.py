from interbotix_ux_modules.arm import InterbotixManipulatorUX

# This script makes the end-effector go to a specific pose (using the modern robotics IK solver) by defining the pose components; No gripper need be attached
#
# To get started, open a terminal and type 'roslaunch interbotix_uxarm_control uxarm_control.launch robot_model:=uxarm6 use_gripper:=false robot_ip:=<robot_ip>'
# Then change to this directory and type 'python ee_pose_components.py'

def main():
    bot = InterbotixManipulatorUX("uxarm6", mode=0, wait_for_finish=True, gripper_type=None)
    bot.arm.go_to_holdup_pose()
    bot.arm.set_ee_pose_components(x=0.2, y=0.1, z=0.2, roll=3.14, pitch=-1.5, yaw=-0.5)
    bot.arm.set_ee_pose_components(x=0.3, z=0.2, roll=3.14, pitch=-1, yaw=0)
    bot.arm.go_to_home_pose()

if __name__=='__main__':
    main()
