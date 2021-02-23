import time
import colorsys
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

# This script uses a color/depth camera to get the arm to find blocks and sort them by color.
# For this demo, the arm is placed to the right of the camera facing outward. When the
# end-effector is located at x=0, y=0.3, z=0.2 w.r.t. the 'wx200/base_link' frame, the AR
# tag should be clearly visible to the camera. Four small baskets should also be placed in front and to the right of the arm.
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=wx200'
# Then change to this directory and type 'python color_sorter.py'

def main():
    # Initialize the arm module along with the pointcloud and armtag modules
    bot = InterbotixManipulatorXS("wx200", moving_time=1.5, accel_time=0.75)
    pcl = InterbotixPointCloudInterface()
    armtag = InterbotixArmTagInterface()

    # set initial arm and gripper pose
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
    # sort them from max to min 'x' position w.r.t. the 'wx200/base_link' frame
    success, clusters = pcl.get_cluster_positions(ref_frame="wx200/base_link", sort_axis="y", reverse=True)

    # pick up all the objects and drop them in baskets
    for cluster in clusters:
        x, y, z = cluster["position"]
        bot.arm.set_ee_pose_components(x=x, y=y, z=0.1, pitch=0.5)
        bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.02, pitch=0.5)
        bot.gripper.close()
        bot.arm.set_ee_pose_components(x=x, y=y, z=0.1, pitch=0.5)
        clr = color_compare(cluster["color"])
        if (clr == "red"):
            bot.arm.set_ee_pose_components(x=0.24, y=-0.1, z=0.2)
        elif (clr == "yellow"):
            bot.arm.set_ee_pose_components(x=0.38, y=-0.24, z=0.2)
        elif (clr == "purple"):
            bot.arm.set_ee_pose_components(x=0.24, y=-0.24, z=0.2)
        elif (clr == "blue"):
            bot.arm.set_ee_pose_components(x=0.38, y=-0.1, z=0.2)
        else:
            # if color cannot be recognized, then put the block back...
            bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.01, pitch=0.5)
        bot.gripper.open()
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
