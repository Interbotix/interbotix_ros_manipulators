import math
import rospy
from threading import Thread
from interbotix_xs_modules.arm import InterbotixManipulatorXS

# This script is used to make two WidowX200 arms work in tandem with one another
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_dual xsarm_dual.launch'
# Then change to this directory and type 'python xsarm_dual.py'
# Note that the 'robot_name' argument used when instantiating an InterbotixManipulatorXS instance
# is the same name as the 'robot_name_X' launch file argument

def robot_1():
    robot_1 = InterbotixManipulatorXS(robot_model="wx200", robot_name="arm_1", moving_time=1.0, gripper_pressure=1.0, init_node=False)
    robot_1.arm.set_ee_pose_components(x=0.3, z=0.2)
    robot_1.gripper.open(delay=0.05)
    robot_1.arm.set_single_joint_position("waist", math.pi/4.0)
    robot_1.gripper.close(delay=0.05)
    robot_1.arm.set_single_joint_position("waist", 0)
    robot_1.arm.go_to_sleep_pose()

def robot_2():
    robot_2 = InterbotixManipulatorXS(robot_model="wx200", robot_name="arm_2", moving_time=1.0, gripper_pressure=1.0, init_node=False)
    robot_2.arm.set_ee_pose_components(x=0.3, z=0.2)
    robot_2.gripper.open(delay=0.05)
    robot_2.arm.set_single_joint_position("waist", -math.pi/4.0)
    robot_2.gripper.close(delay=0.05)
    robot_2.arm.set_single_joint_position("waist", 0)
    robot_2.arm.go_to_sleep_pose()

def main():
    rospy.init_node("xsarm_dual")
    Thread(target=robot_1).start()
    Thread(target=robot_2).start()

if __name__=='__main__':
    main()
