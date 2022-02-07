#include <unordered_map>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "interbotix_xs_msgs/RegisterValues.h"
#include "interbotix_xs_msgs/JointTemps.h"

sensor_msgs::JointState joint_states;

/// @brief Get the current joint states
/// @details - only needed to find initial joint states
void joint_state_cb(const sensor_msgs::JointState &msg)
{
  joint_states = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xsarm_diagnostic_listener");
  ros::NodeHandle nh;

  std::vector<std::string> observe_joints_array;
  if (!nh.getParam("observed_joints", observe_joints_array))
  {
    ROS_ERROR("[xsarm_diagnostic_listener] Failed to get observed_joints parameter. Exiting.");
    exit(1);
  }

  // init pubs, subs, and services
  ros::Publisher pub_group_temp = nh.advertise<interbotix_xs_msgs::JointTemps>("temperatures/joint_group", 1);
  ros::Subscriber sub_joint_states = nh.subscribe("joint_states", 1, joint_state_cb);
  ros::ServiceClient srv_get_reg = nh.serviceClient<interbotix_xs_msgs::RegisterValues>("get_motor_registers");

  // Set loop arbitrarily to 10 Hz
  int loop_hz = 10;
  ros::Rate loop_rate(loop_hz);
  
  // Temperature changes slowly - Publish every 5 seconds
  int temp_freq = 5*loop_hz;
  
  // Wait until a message appears on the joint_states topic
  ros::Duration duration_to_wait_s = ros::Duration(10);
  ros::Time time_start = ros::Time::now();
  while (
    joint_states.position.size() == 0 && 
    ros::ok() && 
    (ros::Time::now() - time_start < duration_to_wait_s))
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  if (joint_states.position.size() == 0)
  {
    ROS_ERROR(
      "[xsarm_diagnostic_listener] Failed to find joint_states under the namespace '%s' after %i seconds. Exiting.",
      nh.getNamespace().c_str(),
      duration_to_wait_s.sec);
      exit(1);
  }

  // Wait until the 'interbotix_xs_sdk' services are up
  // The 'srv_get_reg' service is needed to poll the joint temperatures
  srv_get_reg.waitForExistence();

  // Get the observed joint index
  std::vector<size_t> observe_joint_indexes(observe_joints_array.size());
  ROS_INFO("[xsarm_diagnostic_listener] Listening to the following joints:");
  for (auto joint_name : observe_joints_array)
  {
    ROS_INFO("[xsarm_diagnostic_listener] - %s", joint_name.c_str());
  }

  // Joint state information is no longer needed
  sub_joint_states.shutdown();

  ROS_INFO("[xsarm_diagnostic_listener] Now publishing to observation topics.");

  interbotix_xs_msgs::RegisterValues get_reg_msg;
  get_reg_msg.request.cmd_type = "single";
  get_reg_msg.request.reg = "Present_Temperature";

  int cntr = 0;
  while (ros::ok())
  {
    // Publish the current joint temperatures at temp_freq
    if (cntr % temp_freq == 0)
    {
      cntr = 0;
      interbotix_xs_msgs::JointTemps group_temp_msg;
      for (auto joint_name : observe_joints_array)
      {
        // Call the get_registers service to get present temperature
        get_reg_msg.request.name = joint_name;
        srv_get_reg.call(get_reg_msg);
        group_temp_msg.names.push_back(joint_name);
        group_temp_msg.temps.push_back(get_reg_msg.response.values.at(0));
      }
      // Publish present temperatures
      pub_group_temp.publish(group_temp_msg);
    }
    cntr++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
