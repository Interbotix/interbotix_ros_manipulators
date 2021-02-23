import math
import time
import colorsys
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

# This script uses a color/depth camera to get the arm to find blocks and sort them by color (yellow and blue in this case).
# For this demo, the arm is placed to the left of the camera facing outward. When the
# end-effector is located at x=0, y=-0.3, z=0.2 w.r.t. the 'wx200/base_link' frame, the AR
# tag should be clearly visible to the camera. The arm will form two lines of blocks for each color.
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=wx200'
# Then change to this directory and type 'python block_organizer.py'

def main():
    # Initialize the arm module along with the pointcloud and armtag modules
    bot = InterbotixManipulatorXS("wx250s")
    bot.dxl.robot_set_motor_registers("single", "shoulder", "Position_P_Gain", 1500)
    bot.dxl.robot_set_motor_registers("single", "elbow", "Position_P_Gain", 1500)
    bot.dxl.robot_set_motor_registers("single", "waist", "Position_P_Gain", 1500)
    pcl = InterbotixPointCloudInterface()
    armtag = InterbotixArmTagInterface()

    # set initial arm and gripper pose
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.gripper.open()

    # get the ArmTag pose
    bot.arm.set_ee_pose_components(y=0.3, z=0.2)
    time.sleep(0.5)
    armtag.find_ref_to_arm_base_transform()
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)

    # get the cluster positions
    # sort them from max to min 'y' position w.r.t. the 'wx250s/base_link' frame
    success, clusters = pcl.get_cluster_positions(ref_frame="wx250s/base_link", sort_axis="y", reverse=True)
    color_x_dict = {}
    # pick up all the objects and sort them into two different lines depending on their color
    for cluster in clusters:
        x, y, z = cluster["position"]
        z_dist = 0.2 - (z + 0.02)
        bot.arm.set_ee_pose_components(x=x, y=y, z=0.2, pitch=0.7)
        bot.arm.set_ee_cartesian_trajectory(z=-z_dist, y=0.01)
        bot.gripper.close()
        bot.arm.set_ee_cartesian_trajectory(z=z_dist, y=-0.01)

        clr = color_compare(cluster["color"])
        if clr not in color_x_dict:
            color_x_dict[clr] = -0.14
        else:
            color_x_dict[clr] += 0.07
        if (clr == "yellow"):
            y = -0.25
        elif (clr == "blue"):
            y = -0.35

        bot.arm.set_ee_pose_components(x=color_x_dict[clr], y=y, z=0.2, pitch=0.7, yaw=-math.pi/2.0)
        bot.arm.set_ee_cartesian_trajectory(z=-z_dist)
        bot.gripper.open()
        bot.arm.set_ee_cartesian_trajectory(z=z_dist)

    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.go_to_sleep_pose()

# determines the color of each object using the Hue value in the HSV color space
def color_compare(rgb):
    r,g,b = [x/255.0 for x in rgb]
    h,s,v = colorsys.rgb_to_hsv(r,g,b)

    if h < 0.025: color = "red"
    elif 0.025 < h < 0.05: color = "orange"
    elif 0.1 < h < 0.15: color = "yellow"
    elif 0.3 < h < 0.4: color = "green"
    elif 0.55 < h < 0.65: color = "blue"
    elif 0.75 < h < 0.85: color = "purple"
    else: color = "unknown"
    return color

if __name__=='__main__':
    main()
