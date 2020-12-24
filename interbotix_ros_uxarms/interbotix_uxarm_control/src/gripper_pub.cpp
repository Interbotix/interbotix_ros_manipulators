#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "xarm_msgs/GripperState.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "gripper_pub");
  ros::NodeHandle nh;

  ros::Publisher pub_gripper = nh.advertise<sensor_msgs::JointState>("gripper/joint_state" ,1);
  ros::ServiceClient srv_gripper_state = nh.serviceClient<xarm_msgs::GripperState>("gripper_state");
  srv_gripper_state.waitForExistence();

  int gripper_pub_freq = 5;
  ros::param::get("~gripper_pub_freq", gripper_pub_freq);
  ros::Rate loop_rate(gripper_pub_freq);

  sensor_msgs::JointState msg;
  msg.name.push_back("drive_joint");
  msg.position.push_back(0);
  msg.velocity.push_back(0);
  msg.effort.push_back(0);

  xarm_msgs::GripperState gripper_state_msg;
  while (ros::ok())
  {
    srv_gripper_state.call(gripper_state_msg);
    msg.position.at(0) = 0.85 - gripper_state_msg.response.curr_pos / 1000.0;
    msg.header.stamp = ros::Time::now();
    pub_gripper.publish(msg);
    loop_rate.sleep();
  }

  return 0;
}
