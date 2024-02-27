// Copyright 2024 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "interbotix_xs_msgs/msg/joint_single_command.hpp"
#include "interbotix_xs_msgs/srv/robot_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using interbotix_xs_msgs::msg::JointGroupCommand;
using interbotix_xs_msgs::msg::JointSingleCommand;
using interbotix_xs_msgs::srv::RobotInfo;
using sensor_msgs::msg::JointState;

// globally available joint_states message
JointState::SharedPtr g_joint_states = std::make_shared<JointState>();

/**
 * @brief Joint state callback function
 * @param msg updated joint states
 */
void joint_state_cb(JointState::SharedPtr msg)
{
  g_joint_states = msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("xsarm_puppet_single");

  node->declare_parameter<std::string>("robot_name_leader", "leader");
  node->declare_parameter<std::string>("robot_name_follower", "follower");

  std::string robot_name_leader, robot_name_follower;

  node->get_parameter("robot_name_leader", robot_name_leader);
  node->get_parameter("robot_name_follower", robot_name_follower);

  // Subscribe to the leader robot's joint states and publish those states as joint commands to the
  // follower robot
  auto sub_positions = node->create_subscription<JointState>(
    robot_name_leader + "/joint_states",
    1,
    joint_state_cb);

  auto pub_group = node->create_publisher<JointGroupCommand>(
    robot_name_follower + "/commands/joint_group",
    1);
  auto pub_single = node->create_publisher<JointSingleCommand>(
    robot_name_follower + "/commands/joint_single",
    1);

  // Get some robot info to figure out how many joints the robot has
  auto client_robot_info = node->create_client<RobotInfo>(robot_name_leader + "/get_robot_info");

  rclcpp::Rate loop_rate(30);
  bool success;

  // Wait for the 'arm_node' to finish initializing
  while (
    (pub_group->get_subscription_count() < 1 || g_joint_states->position.size() < 1) &&
    rclcpp::ok())
  {
    rclcpp::spin_some(node->get_node_base_interface());
    loop_rate.sleep();
  }

  // Get info about the 'arm' group from the first robot
  RobotInfo::Response::SharedPtr res_arm_info;
  auto req_arm_info = std::make_shared<RobotInfo::Request>();
  req_arm_info->cmd_type = "group";
  req_arm_info->name = "arm";
  auto response_arm = client_robot_info->async_send_request(req_arm_info);
  if (rclcpp::spin_until_future_complete(
      node->get_node_base_interface(), response_arm) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Could not get info on the '%s' group.",
      req_arm_info->name.c_str());
    return 1;
  }
  res_arm_info = response_arm.get();

  // Get gripper info from the first robot
  RobotInfo::Response::SharedPtr res_gripper_info;
  auto req_gripper_info = std::make_shared<RobotInfo::Request>();
  req_gripper_info->cmd_type = "single";
  req_gripper_info->name = "gripper";
  auto response_gripper = client_robot_info->async_send_request(req_gripper_info);
  if (rclcpp::spin_until_future_complete(
      node->get_node_base_interface(), response_gripper) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Could not get info on the '%s' joint.",
      req_gripper_info->name.c_str());
    return 1;
  }
  res_gripper_info = response_gripper.get();

  RCLCPP_INFO(node->get_logger(), "Ready to start puppetting!");

  while (rclcpp::ok()) {
    // put joint positions from the first robot as position commands for the second robot
    auto pos_msg = JointGroupCommand();
    pos_msg.name = "arm";
    for (auto const & index : res_arm_info->joint_state_indices) {
      pos_msg.cmd.push_back(g_joint_states->position.at(index));
    }
    pub_group->publish(pos_msg);

    // same thing, but now for the gripper
    auto single_msg = JointSingleCommand();
    single_msg.name = "gripper";
    single_msg.cmd = g_joint_states->position.at(res_gripper_info->joint_state_indices.at(0)) * 2;
    pub_single->publish(single_msg);

    rclcpp::spin_some(node->get_node_base_interface());
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
