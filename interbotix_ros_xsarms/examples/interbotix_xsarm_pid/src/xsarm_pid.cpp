#include <ros/ros.h>
#include <unordered_map>
#include "interbotix_xsarm_pid/pid.h"
#include "interbotix_xs_sdk/RobotInfo.h"
#include "interbotix_xs_sdk/JointGroupCommand.h"
#include <sensor_msgs/JointState.h>

MultiPID pid_cntlrs;
ros::Publisher pub_group;
sensor_msgs::JointState joint_states;
interbotix_xs_sdk::RobotInfo group_info;
std::unordered_map<std::string, size_t> js_index_map;
/// @brief ROS Subscriber callback function to get joint states
/// @param msg - most up-to-date joint state message
void joint_state_cb(const sensor_msgs::JointState &msg)
{
  joint_states = msg;
}

/// @brief ROS Timer callback function to run the joint controllers
void pid_controller(const ros::TimerEvent&)
{
  std::vector<double> group_states;
  std::vector<double> group_command(group_info.response.num_joints, 0);
  for (auto const& name : group_info.response.joint_names)
    group_states.push_back(joint_states.position.at(js_index_map[name]));
  pid_cntlrs.multi_pid_compute_control(group_command.data(), group_states);
  interbotix_xs_sdk::JointGroupCommand msg;
  msg.name = "arm";
  for (auto const& value : group_command)
    msg.cmd.push_back(value);
  pub_group.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xsarm_pid");

  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
  std::string control_mode;
  std::vector<double> kp;                 // Proportional gain for each joint in the robot
  std::vector<double> ki;                 // Integral gain for each joint in the robot
  std::vector<double> kd;                 // Derivative gain for each joint in the robot
  std::vector<double> u_min;
  std::vector<double> u_max;
  // get the desired control mode (pwm/current)
  ros::param::param<std::string>("~control_mode", control_mode, "pwm");
  // get the gains and max control effort from the parameter server
  ros::param::get(control_mode + "/kp", kp);
  ros::param::get(control_mode + "/ki", ki);
  ros::param::get(control_mode + "/kd", kd);
  ros::param::get(control_mode + "/u_min", u_min);
  ros::param::get(control_mode + "/u_max", u_max);
  ros::Subscriber sub_joint_states = nh.subscribe("joint_states", 1, joint_state_cb);
  pub_group = nh.advertise<interbotix_xs_sdk::JointGroupCommand>("commands/joint_group", 1);
  ros::ServiceClient srv_robot_info = nh.serviceClient<interbotix_xs_sdk::RobotInfo>("get_robot_info");
  srv_robot_info.waitForExistence();
  group_info.request.cmd_type = "group";
  group_info.request.name = "arm";
  srv_robot_info.call(group_info);

  pid_cntlrs.multi_pid_init(group_info.response.num_joints, kp, ki, kd, u_min, u_max);
  std::vector<double> home_positions(group_info.response.num_joints, 0);
  pid_cntlrs.multi_pid_set_refs(home_positions);

  // Wait until the first joint_states message is received
  while (joint_states.position.size() == 0 && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  for (size_t i{0}; i < joint_states.name.size(); i++)
    js_index_map.insert({joint_states.name.at(i), i});
  ros::Timer tmr_pid_controller = nh.createTimer(ros::Duration(1/10), pid_controller);

  double time_start;

  // Give 10 seconds for the robot arm to settle at reference positions
  time_start = ros::Time::now().toSec();
  while(ros::ok() && ros::Time::now().toSec() < (time_start + 10))
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Initialize the reference positions for each joint to their respective 'sleep' positions
  std::vector<double> sleep_positions(group_info.response.joint_sleep_positions.begin(), group_info.response.joint_sleep_positions.end());
  pid_cntlrs.multi_pid_set_refs(sleep_positions);

  // Give 10 seconds for the robot arm to settle at reference positions
  time_start = ros::Time::now().toSec();
  while(ros::ok() && ros::Time::now().toSec() < (time_start + 10))
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Command all joints to '0' pwm/current, essentially torquing them off
  interbotix_xs_sdk::JointGroupCommand msg;
  msg.name = "arm";
  msg.cmd = std::vector<float>(group_info.response.num_joints, 0);
  pub_group.publish(msg);
  return 0;
}
