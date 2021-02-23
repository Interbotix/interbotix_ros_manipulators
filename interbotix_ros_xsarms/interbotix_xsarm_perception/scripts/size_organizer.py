import math
import time
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

# This script uses a color/depth camera to get the arm to find different sized objects and sort them by size.
# For this demo, the arm is placed to the right of the camera facing outward. When the
# end-effector is located at x=0, y=0.3, z=0.2 w.r.t. the 'wx250s/base_link' frame, the AR
# tag should be clearly visible to the camera. Objects will be placed in a single row on the right side of the robot
# in descending size.
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=wx200'
# Then change to this directory and type 'python block_organizer.py'

def main():
    # Initialize the arm module along with the pointcloud and armtag modules
    bot = InterbotixManipulatorXS("wx250s")
    pcl = InterbotixPointCloudInterface()
    armtag = InterbotixArmTagInterface()

    # set the initial arm and gripper pose
    bot.dxl.robot_set_motor_registers("single", "waist", "Position_P_Gain", 1500)
    bot.dxl.robot_set_motor_registers("single", "shoulder", "Position_P_Gain", 1500)
    bot.dxl.robot_set_motor_registers("single", "elbow", "Position_P_Gain", 1500)
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.gripper.open()

    # get the ArmTag pose
    bot.arm.set_ee_pose_components(y=0.3, z=0.2)
    time.sleep(0.5)
    armtag.find_ref_to_arm_base_transform()
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)

    # get the cluster positions
    # sort them from min to max 'y' position w.r.t. the 'wx250s/base_link' frame
    success, clusters = pcl.get_cluster_positions(ref_frame="wx250s/base_link", sort_axis="y")

    # sort clusters by their size - from most points to min points
    sorted_clusters = sorted(clusters, key=lambda cluster : cluster["num_points"], reverse=True)

    # pick up all the objects and place in a single row based on size
    x_des = -0.07
    for cluster in sorted_clusters:
        x, y, z = cluster["position"]
        z_dist = 0.2 - (z + 0.01)
        bot.arm.set_ee_pose_components(x=x, y=y, z=0.2, pitch=0.5)
        bot.arm.set_ee_cartesian_trajectory(z=-z_dist)
        bot.gripper.close()
        bot.arm.set_ee_cartesian_trajectory(z=z_dist)
        bot.arm.set_ee_pose_components(x=x_des, y=-0.3, z=0.2, pitch=0.5, yaw=-math.pi/2)
        bot.arm.set_ee_cartesian_trajectory(z=-z_dist)
        bot.gripper.open()
        bot.arm.set_ee_cartesian_trajectory(z=z_dist)
        x_des += 0.07
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
