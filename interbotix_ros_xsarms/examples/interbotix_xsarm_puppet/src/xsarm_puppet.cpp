#include <ros/ros.h>
#include "interbotix_xs_sdk/JointGroupCommand.h"
#include "interbotix_xs_sdk/JointSingleCommand.h"
#include "interbotix_xs_sdk/RobotInfo.h"
#include <sensor_msgs/JointState.h>

sensor_msgs::JointState joint_states;         // globally available joint_states message

/// @brief Joint state callback function
/// @param msg - updated joint states
void joint_state_cb(const sensor_msgs::JointState &msg)
{
  joint_states = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xsarm_puppet");
  ros::NodeHandle n;
  std::string robot_name_master, robot_name_puppet;
  ros::param::get("~robot_name_master", robot_name_master);
  ros::param::get("~robot_name_puppet", robot_name_puppet);

  // Subscribe to the first robot's joint states and publish those states as joint commands to the second robot
  ros::Subscriber sub_positions = n.subscribe(robot_name_master + "/joint_states", 1, joint_state_cb);
  ros::Publisher pub_group = n.advertise<interbotix_xs_sdk::JointGroupCommand>(robot_name_puppet + "/commands/joint_group", 1);
  ros::Publisher pub_single = n.advertise<interbotix_xs_sdk::JointSingleCommand>(robot_name_puppet + "/commands/joint_single", 1);

  // Get some robot info to figure out how many joints the robot has
  ros::ServiceClient srv_robot_info = n.serviceClient<interbotix_xs_sdk::RobotInfo>(robot_name_master + "/get_robot_info");

  ros::Rate loop_rate(30);
  bool success;

  // Wait for the 'arm_node' to finish initializing
  while ((pub_group.getNumSubscribers() < 1 || joint_states.position.size() < 1) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Get info about the 'arm' group from the first robot
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
    // put joint positions from the first robot as position commands for the second robot
    interbotix_xs_sdk::JointGroupCommand pos_msg;
    pos_msg.name = "arm";
    for (auto const& index : arm_info_srv.response.joint_state_indices)
      pos_msg.cmd.push_back(joint_states.position.at(index));
    pub_group.publish(pos_msg);

    // same thing, but now for the gripper
    interbotix_xs_sdk::JointSingleCommand single_msg;
    single_msg.name = "gripper";
    single_msg.cmd = joint_states.position.at(gripper_info_srv.response.joint_state_indices.at(0))*2;
    pub_single.publish(single_msg);
    cntr = 0;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
