#include <cstdlib>
#include <unordered_map>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include "interbotix_xs_sdk/JointGroupCommand.h"
#include "interbotix_xs_sdk/JointSingleCommand.h"
#include "interbotix_xs_sdk/RobotInfo.h"
#include "interbotix_xs_sdk/RegisterValues.h"
#include "interbotix_xsarm_diagnostic_tool/JointTemps.h"

static const float PI = 3.14159265358979f;
sensor_msgs::JointState joint_states;

/// @brief Get the current joint states
/// @details - only needed to find initial joint states
void joint_state_cb(const sensor_msgs::JointState &msg)
{
  joint_states = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xsarm_diagnostic_tool");
  ros::NodeHandle nh;
  ros::Publisher pub_group = nh.advertise<interbotix_xs_sdk::JointGroupCommand>("commands/joint_group", 1);
  ros::Publisher pub_joint = nh.advertise<interbotix_xs_sdk::JointSingleCommand>("commands/joint_single", 1);
  ros::Publisher pub_state = nh.advertise<sensor_msgs::JointState>("states/joint_observe", 1);
  ros::Publisher pub_single_temp = nh.advertise<std_msgs::Int32>("temperatures/joint_observe", 1);
  ros::Publisher pub_group_temp = nh.advertise<interbotix_xsarm_diagnostic_tool::JointTemps>("temperatures/joint_group", 1);
  ros::Subscriber sub_joint_states = nh.subscribe("joint_states", 1, joint_state_cb);
  ros::ServiceClient srv_robot_info = nh.serviceClient<interbotix_xs_sdk::RobotInfo>("get_robot_info");
  ros::ServiceClient srv_set_reg = nh.serviceClient<interbotix_xs_sdk::RegisterValues>("set_motor_registers");
  ros::ServiceClient srv_get_reg = nh.serviceClient<interbotix_xs_sdk::RegisterValues>("get_motor_registers");

  // publish joint position commands arbitrarily at 10 Hz
  int loop_hz = 10;
  ros::Rate loop_rate(loop_hz);
  // Temperature does not change so much over time...so only publish the current temp every 5 seconds
  int temp_freq = 5*loop_hz;
  // Wait until a message appears on the joint_states topic
  while (joint_states.position.size() == 0 && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Wait until the 'interbotix_xs_sdk' services are up
  // The 'srv_robot_info' service is needed to get the lower and upper joint limits while
  // The 'srv_set_reg' service is needed to adjust the 'Profile_Velocity' register at the end of the test
  // the 'srv_get_reg' service is needed to poll the joint temperatures
  srv_robot_info.waitForExistence();
  srv_set_reg.waitForExistence();
  srv_get_reg.waitForExistence();
  // Get the number of seconds to run the test for
  int test_duration = 600;
  ros::param::get("~test_duration", test_duration);
  // Get the name of the joint that the user would like to move
  std::string cmd_joint, observe_joint;
  ros::param::get("~cmd_joint", cmd_joint);
  ros::param::get("~observe_joint", observe_joint);
  interbotix_xs_sdk::RobotInfo srv_joint_info, srv_group_info;
  srv_joint_info.request.cmd_type = "single";
  srv_joint_info.request.name = cmd_joint;
  srv_group_info.request.cmd_type = "group";
  srv_group_info.request.name = "arm";
  srv_robot_info.call(srv_joint_info);
  srv_robot_info.call(srv_group_info);


  // Since the motion of the joint will follow a sinusoidal path symmetric around '0' radians, determine which of the limits are closer to '0' and set that
  // as the max angular position of the path
  double upper_limit = srv_joint_info.response.joint_upper_limits.at(0);
  double abs_lower_limit = fabs(srv_joint_info.response.joint_lower_limits.at(0));
  double pos_limit = std::min(upper_limit, abs_lower_limit);

  std::unordered_map<std::string, size_t> js_index_map, cmd_index_map;
  for (size_t i{0}; i < joint_states.name.size(); i++)
    js_index_map.insert({joint_states.name.at(i), i});
  for (size_t i{0}; i < srv_group_info.response.joint_names.size(); i++)
    cmd_index_map.insert({srv_group_info.response.joint_names.at(i), i});

  // Give a second for the user to let go of the robot arm after it is torqued on and holding its position
  ros::Duration(1.0).sleep();
  interbotix_xs_sdk::JointSingleCommand initial_pos_msg;
  initial_pos_msg.name = cmd_joint;
  initial_pos_msg.cmd = 0;
  pub_joint.publish(initial_pos_msg);
  // Give a second for the joint that will be moving during the test to get to its zero position.
  ros::Duration(1.0).sleep();
  // Now loop for 10 minutes while commanding the 'moving' joint
  int cntr = 0;
  double time_start = ros::Time::now().toSec();
  while (ros::ok() && ros::Time::now().toSec() < (time_start + test_duration))
  {
    interbotix_xs_sdk::JointSingleCommand msg;
    msg.name = cmd_joint;
    sensor_msgs::JointState js_msg;
    js_msg.header.stamp = ros::Time::now();
    js_msg.name.push_back(observe_joint);
    js_msg.position.push_back(joint_states.position.at(js_index_map[observe_joint]));
    js_msg.velocity.push_back(joint_states.velocity.at(js_index_map[observe_joint]));
    js_msg.effort.push_back(joint_states.effort.at(js_index_map[observe_joint]));
    // Sinusoidal trajectory for the joint to follow
    double pos = pos_limit*sin(PI/4.0*(ros::Time::now().toSec() - time_start));
    msg.cmd = pos;
    pub_joint.publish(msg);
    pub_state.publish(js_msg);
    // Every 500 iterations or 5 seconds, publish the current joint temperatures
    if (cntr % temp_freq == 0)
    {
      cntr = 0;
      interbotix_xs_sdk::RegisterValues get_reg_msg;
      get_reg_msg.request.cmd_type = "group";
      get_reg_msg.request.name = "arm";
      get_reg_msg.request.reg = "Present_Temperature";
      srv_get_reg.call(get_reg_msg);
      std_msgs::Int32 temp_msg;
      interbotix_xsarm_diagnostic_tool::JointTemps group_temp_msg;
      temp_msg.data = get_reg_msg.response.values.at(cmd_index_map[observe_joint]);
      group_temp_msg.names = srv_group_info.response.joint_names;
      group_temp_msg.temps = get_reg_msg.response.values;
      pub_single_temp.publish(temp_msg);
      pub_group_temp.publish(group_temp_msg);
    }
    cntr++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  // command arm to go to its sleep pose
  interbotix_xs_sdk::RegisterValues set_reg_msg;
  set_reg_msg.request.cmd_type = "group";
  set_reg_msg.request.name = "arm";
  set_reg_msg.request.reg = "Profile_Velocity";
  set_reg_msg.request.value = 30;
  srv_set_reg.call(set_reg_msg);
  interbotix_xs_sdk::JointGroupCommand msg;
  msg.name = "arm";
  msg.cmd = srv_group_info.response.joint_sleep_positions;
  pub_group.publish(msg);
  return 0;
}
