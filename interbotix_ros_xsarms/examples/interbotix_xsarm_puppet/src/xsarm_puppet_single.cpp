#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "interbotix_xs_sdk/JointGroupCommand.h"
#include "interbotix_xs_sdk/JointSingleCommand.h"
#include "interbotix_xs_sdk/RobotInfo.h"

sensor_msgs::JointState joint_states;         // globally available joint_states message

/// @brief Joint state callback function
/// @param msg - updated joint states
void joint_state_cb(const sensor_msgs::JointState &msg)
{
  joint_states = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xsarm_puppet_single");
  ros::NodeHandle n;

  // Subscribe to the robot's joint states and publish those states as joint commands so that rosbag can record them
  ros::Subscriber sub_positions = n.subscribe("joint_states", 1, joint_state_cb);
  ros::Publisher pub_group = n.advertise<interbotix_xs_sdk::JointGroupCommand>("commands/joint_group", 1);
  ros::Publisher pub_single = n.advertise<interbotix_xs_sdk::JointSingleCommand>("commands/joint_single", 1);
  // Get some robot info to figure out how many joints the robot has
  ros::ServiceClient srv_robot_info = n.serviceClient<interbotix_xs_sdk::RobotInfo>("get_robot_info");

  ros::Rate loop_rate(50);
  bool success;

  // Wait for the 'arm_node' to finish initializing
  while ((pub_group.getNumSubscribers() < 1 || joint_states.position.size() < 1) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Get the number of joints that the first robot has
  interbotix_xs_sdk::RobotInfo arm_info_srv;
  arm_info_srv.request.cmd_type = "group";
  arm_info_srv.request.name = "arm";
  success = srv_robot_info.call(arm_info_srv);
  if (!success)
  {
    ROS_ERROR("Could not get info on the 'arm' group.");
    return 1;
  }

  // Get gripper info from the first robot
  interbotix_xs_sdk::RobotInfo gripper_info_srv;
  gripper_info_srv.request.cmd_type = "single";
  gripper_info_srv.request.name = "gripper";
  success = srv_robot_info.call(gripper_info_srv);
  if (!success)
  {
    ROS_ERROR("Could not get info on the 'gripper' joint.");
    return 1;
  }

  size_t cntr = 0;
  while (ros::ok())
  {
    // put joint positions from the robot as position commands for itself
    interbotix_xs_sdk::JointGroupCommand pos_msg;
    pos_msg.name = "arm";
    for (auto const& index : arm_info_srv.response.joint_state_indices)
      pos_msg.cmd.push_back(joint_states.position.at(index));
    pub_group.publish(pos_msg);

    interbotix_xs_sdk::JointSingleCommand single_msg;
    single_msg.name = "gripper";
    single_msg.cmd = joint_states.position.at(gripper_info_srv.response.joint_state_indices.at(0))*2;
    pub_single.publish(single_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
