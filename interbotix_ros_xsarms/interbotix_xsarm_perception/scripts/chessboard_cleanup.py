import time
import colorsys
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

# This script uses a color/depth camera to get a VX300S arm to find chess pieces on a board and sort them by color.
# For this demo, the arm is placed at a 90 degree angle to the left of the camera. When the
# end-effector is located at x=0.5, y=-0.1 z=0.2 w.r.t. the 'vx300s/base_link' frame, the AR
# tag should be clearly visible to the camera. Two small containers should be placed on either side of the board.
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=vx300s'
# Then change to this directory and type 'python chessboard_cleanup.py'

def main():
    # Initialize the arm module along with the pointcloud and armtag modules
    bot = InterbotixManipulatorXS("vx300s", moving_time=1.5, accel_time=0.75, gripper_pressure=1.0)
    pcl = InterbotixPointCloudInterface()
    armtag = InterbotixArmTagInterface()

    # get the ArmTag pose
    bot.arm.set_ee_pose_components(x=0.5, z=0.2)
    bot.arm.set_ee_pose_components(x=0.5, y=-0.1, z=0.2)
    time.sleep(0.5)
    armtag.find_ref_to_arm_base_transform()
    bot.arm.set_ee_pose_components(x=0.5, z=0.2)
    bot.arm.go_to_sleep_pose()

    # get the cluster positions
    # sort them from min to max 'x' position w.r.t. the 'vx300s/base_link' frame
    success, clusters = pcl.get_cluster_positions(ref_frame="vx300s/base_link", sort_axis="x", reverse=False)

    # set initial arm and gripper pose
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.gripper.open()

    # pick up all the chess pieces and place them in the right drawer based on color
    for cluster in clusters:
        x, y, z = cluster["position"]
        bot.arm.set_ee_pose_components(x=x, y=y, z=0.1, yaw=0)
        bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.02, yaw=0)
        bot.gripper.close()
        bot.arm.set_ee_pose_components(x=x, y=y, z=0.1, yaw=0)
        clr = color_compare(cluster["color"])
        if (clr == "white"):
            bot.arm.set_ee_pose_components(x=0.5, y=-0.33, z=0.1, yaw=0)
        elif (clr == "black"):
            bot.arm.set_ee_pose_components(x=0.5, y=0.33, z=0.1, yaw=0)
        bot.gripper.open()
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.go_to_sleep_pose()

# determines the color of each object using the Value value in the HSV color space
# the Value is used since it does a much better job differentiating between black/white pieces than Hue does
def color_compare(rgb):
    r,g,b = [x/255.0 for x in rgb]
    h,s,v = colorsys.rgb_to_hsv(r,g,b)

    if v < 0.3: color = "black"
    else: color = "white"

    return color

if __name__=='__main__':
    main()
