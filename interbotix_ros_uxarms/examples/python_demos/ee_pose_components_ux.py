from interbotix_ux_modules.core import InterbotixRobotUXCore

# This script makes the end-effector go to a specific pose (using the built-in IK solver) by defining the pose components; No gripper need be attached
#
# To get started, open a terminal and type 'roslaunch interbotix_uxarm_control uxarm_control.launch robot_model:=uxarm6 use_gripper:=false robot_ip:=<robot_ip>'
# Then change to this directory and type 'python ee_pose_components.py'

def main():
    bot = InterbotixRobotUXCore("uxarm6", mode=0, wait_for_finish=True, gripper_type=None)
    bot.robot_move_line(pose=[300, 100, 200, 3.14, -1.5, -0.5], vel=250, accel=1000)
    bot.robot_move_joint(cmd=[-1.0, -0.5, -0.5, 1.0, 0.7, -0.65], vel=0.5, accel=3.0)
    bot.robot_move_line(pose=[300, 0, 200, 3.14, -1, 0], vel=150, accel=1500)
    bot.robot_go_home()

if __name__=='__main__':
    main()
