#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "interbotix_xsarm_joy/ArmJoy.h"

// PS3 Controller button mappings
static const std::map<std::string, int> ps3 = {{"GRIPPER_PWM_DEC", 0}, // buttons start here
                                               {"GRIPPER_OPEN", 1},
                                               {"GRIPPER_PWM_INC", 2},
                                               {"GRIPPER_CLOSE", 3},
                                               {"EE_Y_INC", 4},
                                               {"EE_Y_DEC", 5},
                                               {"WAIST_CCW", 6},
                                               {"WAIST_CW", 7},
                                               {"SLEEP_POSE", 8},
                                               {"HOME_POSE", 9},
                                               {"TORQUE_ENABLE", 10},
                                               {"FLIP_EE_X", 11},
                                               {"FLIP_EE_ROLL", 12},
                                               {"SPEED_INC", 13},
                                               {"SPEED_DEC", 14},
                                               {"SPEED_COURSE", 15},
                                               {"SPEED_FINE", 16},
                                               {"EE_X", 0},            // axes start here
                                               {"EE_Z", 1},
                                               {"EE_ROLL", 3},
                                               {"EE_PITCH", 4}};

// PS4 Controller button mappings
static const std::map<std::string, int> ps4 = {{"GRIPPER_PWM_DEC", 0}, // buttons start here
                                               {"GRIPPER_OPEN", 1},
                                               {"GRIPPER_PWM_INC", 2},
                                               {"GRIPPER_CLOSE", 3},
                                               {"EE_Y_INC", 4},
                                               {"EE_Y_DEC", 5},
                                               {"WAIST_CCW", 6},
                                               {"WAIST_CW", 7},
                                               {"SLEEP_POSE", 8},
                                               {"HOME_POSE", 9},
                                               {"TORQUE_ENABLE", 10},
                                               {"FLIP_EE_X", 11},
                                               {"FLIP_EE_ROLL", 12},
                                               {"EE_X", 0},            // axes start here
                                               {"EE_Z", 1},
                                               {"EE_ROLL", 3},
                                               {"EE_PITCH", 4},
                                               {"SPEED_TYPE", 6},
                                               {"SPEED", 7}};

// Xbox 360 Controller button mappings
static const std::map<std::string, int> xbox360 = {{"GRIPPER_PWM_DEC", 0}, // buttons start here
                                                   {"GRIPPER_OPEN", 1},
                                                   {"GRIPPER_CLOSE", 2},
                                                   {"GRIPPER_PWM_INC", 3},
                                                   {"WAIST_CCW", 4},
                                                   {"WAIST_CW", 5},
                                                   {"SLEEP_POSE", 6},
                                                   {"HOME_POSE", 7},
                                                   {"TORQUE_ENABLE", 8},
                                                   {"FLIP_EE_X", 9},
                                                   {"FLIP_EE_ROLL", 10},
                                                   {"EE_X", 0},            // axes start here
                                                   {"EE_Z", 1},
                                                   {"EE_Y_INC", 2},
                                                   {"EE_ROLL", 3},
                                                   {"EE_PITCH", 4},
                                                   {"EE_Y_DEC", 5},
                                                   {"SPEED_TYPE", 6},
                                                   {"SPEED", 7}};


ros::Publisher pub_joy_cmd;                                 // ROS Publisher to publish ArmJoy messages
interbotix_xsarm_joy::ArmJoy prev_joy_cmd;                  // Keep track of the previously commanded ArmJoy message so that only unique messages are published
std::map<std::string, int> cntlr;                           // Holds the controller button mappings
std::string controller_type;                                // Holds the name of the controller received from the ROS Parameter server
double threshold;                                           // Joystick sensitivity threshold

/// @brief Joystick callback to create custom ArmJoy messages to control the Arm
/// @param msg - raw sensor_msgs::Joy data
void joy_state_cb(const sensor_msgs::Joy &msg)
{
  static bool flip_ee_roll_cmd = false;
  static bool flip_ee_roll_cmd_last_state = false;
  static bool flip_ee_x_cmd = false;
  static bool flip_ee_x_cmd_last_state = false;
  static bool flip_torque_cmd = true;
  static bool flip_torque_cmd_last_state = true;
  static double time_start;
  static bool timer_started = false;
  interbotix_xsarm_joy::ArmJoy joy_cmd;

  // Check if the torque_cmd should be flipped
  if (msg.buttons.at(cntlr["TORQUE_ENABLE"]) == 1 && flip_torque_cmd_last_state == false)
  {
    flip_torque_cmd = true;
    joy_cmd.torque_cmd = interbotix_xsarm_joy::ArmJoy::TORQUE_ON;
  }
  else if (msg.buttons.at(cntlr["TORQUE_ENABLE"]) == 1 && flip_torque_cmd_last_state == true)
  {
    time_start = ros::Time::now().toSec();
    timer_started = true;
  }
  else if (msg.buttons.at(cntlr["TORQUE_ENABLE"]) == 0)
  {
    if (timer_started && ros::Time::now().toSec() - time_start > 3)
    {
      joy_cmd.torque_cmd = interbotix_xsarm_joy::ArmJoy::TORQUE_OFF;
      flip_torque_cmd = false;
    }
    flip_torque_cmd_last_state = flip_torque_cmd;
    timer_started = false;
  }

  // Check if the ee_x_cmd should be flipped
  if (msg.buttons.at(cntlr["FLIP_EE_X"]) == 1 && flip_ee_x_cmd_last_state == false)
    flip_ee_x_cmd = true;
  else if (msg.buttons.at(cntlr["FLIP_EE_X"]) == 1 && flip_ee_x_cmd_last_state == true)
    flip_ee_x_cmd = false;
  else if (msg.buttons.at(cntlr["FLIP_EE_X"]) == 0)
    flip_ee_x_cmd_last_state = flip_ee_x_cmd;

  // Check the ee_x_cmd
  if (msg.axes.at(cntlr["EE_X"]) >= threshold && flip_ee_x_cmd == false)
    joy_cmd.ee_x_cmd = interbotix_xsarm_joy::ArmJoy::EE_X_INC;
  else if (msg.axes.at(cntlr["EE_X"]) <= -threshold && flip_ee_x_cmd == false)
    joy_cmd.ee_x_cmd = interbotix_xsarm_joy::ArmJoy::EE_X_DEC;
  else if (msg.axes.at(cntlr["EE_X"]) >= threshold && flip_ee_x_cmd == true)
    joy_cmd.ee_x_cmd = interbotix_xsarm_joy::ArmJoy::EE_X_DEC;
  else if (msg.axes.at(cntlr["EE_X"]) <= -threshold && flip_ee_x_cmd == true)
    joy_cmd.ee_x_cmd = interbotix_xsarm_joy::ArmJoy::EE_X_INC;

  // Check the ee_y_cmd
  if (controller_type == "ps3" || controller_type == "ps4")
  {
    if (msg.buttons.at(cntlr["EE_Y_INC"]) == 1)
      joy_cmd.ee_y_cmd = interbotix_xsarm_joy::ArmJoy::EE_Y_INC;
    else if (msg.buttons.at(cntlr["EE_Y_DEC"]) == 1)
      joy_cmd.ee_y_cmd = interbotix_xsarm_joy::ArmJoy::EE_Y_DEC;
  }
  else if (controller_type == "xbox360")
  {
    if (msg.axes.at(cntlr["EE_Y_INC"]) <= 1.0 - 2.0 * threshold)
      joy_cmd.ee_y_cmd = interbotix_xsarm_joy::ArmJoy::EE_Y_INC;
    else if (msg.axes.at(cntlr["EE_Y_DEC"]) <= 1.0 - 2.0 * threshold)
      joy_cmd.ee_y_cmd = interbotix_xsarm_joy::ArmJoy::EE_Y_DEC;
  }

  // Check the ee_z_cmd
  if (msg.axes.at(cntlr["EE_Z"]) >= threshold)
    joy_cmd.ee_z_cmd = interbotix_xsarm_joy::ArmJoy::EE_Z_INC;
  else if (msg.axes.at(cntlr["EE_Z"]) <= -threshold)
    joy_cmd.ee_z_cmd = interbotix_xsarm_joy::ArmJoy::EE_Z_DEC;

  // Check if the ee_roll_cmd should be flipped
  if (msg.buttons.at(cntlr["FLIP_EE_ROLL"]) == 1 && flip_ee_roll_cmd_last_state == false)
    flip_ee_roll_cmd = true;
  else if (msg.buttons.at(cntlr["FLIP_EE_ROLL"]) == 1 && flip_ee_roll_cmd_last_state == true)
    flip_ee_roll_cmd = false;
  else if (msg.buttons.at(cntlr["FLIP_EE_ROLL"]) == 0)
    flip_ee_roll_cmd_last_state = flip_ee_roll_cmd;

  // Check the ee_roll_cmd
  if (msg.axes.at(cntlr["EE_ROLL"]) >= threshold && flip_ee_roll_cmd == false)
    joy_cmd.ee_roll_cmd = interbotix_xsarm_joy::ArmJoy::EE_ROLL_CW;
  else if (msg.axes.at(cntlr["EE_ROLL"]) <= -threshold && flip_ee_roll_cmd == false)
    joy_cmd.ee_roll_cmd = interbotix_xsarm_joy::ArmJoy::EE_ROLL_CCW;
  else if (msg.axes.at(cntlr["EE_ROLL"]) >= threshold && flip_ee_roll_cmd == true)
    joy_cmd.ee_roll_cmd = interbotix_xsarm_joy::ArmJoy::EE_ROLL_CCW;
  else if (msg.axes.at(cntlr["EE_ROLL"]) <= -threshold && flip_ee_roll_cmd == true)
    joy_cmd.ee_roll_cmd = interbotix_xsarm_joy::ArmJoy::EE_ROLL_CW;

  // Check the ee_pitch_cmd
  if (msg.axes.at(cntlr["EE_PITCH"]) >= threshold)
    joy_cmd.ee_pitch_cmd = interbotix_xsarm_joy::ArmJoy::EE_PITCH_UP;
  else if (msg.axes.at(cntlr["EE_PITCH"]) <= -threshold)
    joy_cmd.ee_pitch_cmd = interbotix_xsarm_joy::ArmJoy::EE_PITCH_DOWN;

  // Check the waist_cmd
  if (msg.buttons.at(cntlr["WAIST_CCW"]) == 1)
    joy_cmd.waist_cmd = interbotix_xsarm_joy::ArmJoy::WAIST_CCW;
  else if (msg.buttons.at(cntlr["WAIST_CW"]) == 1)
    joy_cmd.waist_cmd = interbotix_xsarm_joy::ArmJoy::WAIST_CW;

  // Check the gripper_cmd
  if (msg.buttons.at(cntlr["GRIPPER_CLOSE"]) == 1)
    joy_cmd.gripper_cmd = interbotix_xsarm_joy::ArmJoy::GRIPPER_CLOSE;
  else if (msg.buttons.at(cntlr["GRIPPER_OPEN"]) == 1)
    joy_cmd.gripper_cmd = interbotix_xsarm_joy::ArmJoy::GRIPPER_OPEN;

  // Check the pose_cmd
  if (msg.buttons.at(cntlr["HOME_POSE"]) == 1)
    joy_cmd.pose_cmd = interbotix_xsarm_joy::ArmJoy::HOME_POSE;
  else if (msg.buttons.at(cntlr["SLEEP_POSE"]) == 1)
    joy_cmd.pose_cmd = interbotix_xsarm_joy::ArmJoy::SLEEP_POSE;

  if (controller_type == "ps3")
  {
    // Check the speed_cmd
    if (msg.buttons.at(cntlr["SPEED_INC"]) == 1)
      joy_cmd.speed_cmd = interbotix_xsarm_joy::ArmJoy::SPEED_INC;
    else if (msg.buttons.at(cntlr["SPEED_DEC"]) == 1)
      joy_cmd.speed_cmd = interbotix_xsarm_joy::ArmJoy::SPEED_DEC;

    // Check the speed_toggle_cmd
    if (msg.buttons.at(cntlr["SPEED_COURSE"]) == 1)
      joy_cmd.speed_toggle_cmd = interbotix_xsarm_joy::ArmJoy::SPEED_COURSE;
    else if (msg.buttons.at(cntlr["SPEED_FINE"]) == 1)
      joy_cmd.speed_toggle_cmd = interbotix_xsarm_joy::ArmJoy::SPEED_FINE;
  }
  else if (controller_type == "ps4" || controller_type == "xbox360")
  {
    // Check the speed_cmd
    if (msg.axes.at(cntlr["SPEED"]) == 1)
      joy_cmd.speed_cmd = interbotix_xsarm_joy::ArmJoy::SPEED_INC;
    else if (msg.axes.at(cntlr["SPEED"]) == -1)
      joy_cmd.speed_cmd = interbotix_xsarm_joy::ArmJoy::SPEED_DEC;

    // Check the speed_toggle_cmd
    if (msg.axes.at(cntlr["SPEED_TYPE"]) == 1)
      joy_cmd.speed_toggle_cmd = interbotix_xsarm_joy::ArmJoy::SPEED_COURSE;
    else if (msg.axes.at(cntlr["SPEED_TYPE"]) == -1)
      joy_cmd.speed_toggle_cmd = interbotix_xsarm_joy::ArmJoy::SPEED_FINE;
  }

  // Check the gripper_pwm_cmd
  if (msg.buttons.at(cntlr["GRIPPER_PWM_INC"]) == 1)
    joy_cmd.gripper_pwm_cmd = interbotix_xsarm_joy::ArmJoy::GRIPPER_PWM_INC;
  else if (msg.buttons.at(cntlr["GRIPPER_PWM_DEC"]) == 1)
    joy_cmd.gripper_pwm_cmd = interbotix_xsarm_joy::ArmJoy::GRIPPER_PWM_DEC;

  // Only publish a ArmJoy message if any of the following fields have changed.
  if (!(prev_joy_cmd.ee_x_cmd == joy_cmd.ee_x_cmd &&
      prev_joy_cmd.ee_y_cmd == joy_cmd.ee_y_cmd &&
      prev_joy_cmd.ee_z_cmd == joy_cmd.ee_z_cmd &&
      prev_joy_cmd.ee_roll_cmd == joy_cmd.ee_roll_cmd &&
      prev_joy_cmd.ee_pitch_cmd == joy_cmd.ee_pitch_cmd &&
      prev_joy_cmd.waist_cmd == joy_cmd.waist_cmd &&
      prev_joy_cmd.gripper_cmd == joy_cmd.gripper_cmd &&
      prev_joy_cmd.pose_cmd == joy_cmd.pose_cmd &&
      prev_joy_cmd.speed_cmd == joy_cmd.speed_cmd &&
      prev_joy_cmd.speed_toggle_cmd == joy_cmd.speed_toggle_cmd &&
      prev_joy_cmd.gripper_pwm_cmd == joy_cmd.gripper_pwm_cmd &&
      prev_joy_cmd.torque_cmd == joy_cmd.torque_cmd))
      pub_joy_cmd.publish(joy_cmd);
  prev_joy_cmd = joy_cmd;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xsarm_joy");
  ros::NodeHandle n;
  ros::param::get("~threshold", threshold);
  ros::param::get("~controller", controller_type);
  if (controller_type == "xbox360")
    cntlr = xbox360;
  else if (controller_type == "ps3")
    cntlr = ps3;
  else
    cntlr = ps4;
  ros::Subscriber sub_joy_raw = n.subscribe("commands/joy_raw", 10, joy_state_cb);
  pub_joy_cmd = n.advertise<interbotix_xsarm_joy::ArmJoy>("commands/joy_processed", 10);
  ros::spin();
  return 0;
}
