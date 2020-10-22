#include "interbotix_xsarm_pid/pid.h"

/// @brief Constructor for the PID class
PID::PID(const double Kp, const double Ki, const double Kd, const double u_min, const double u_max)
  : Kp(Kp), Ki(Ki), Kd(Kd), u_min(u_min), u_max(u_max)
{
  pid_clear();
}

/// @brief Compute one iteration of the feedback controller
/// @param ref - value to track
/// @param ff - feedforward term
/// @param actual - observed value
double PID::pid_compute_control(const double ref, const double ff, const double actual)
{
  double error = ref - actual;
  double i_error_temp = i_error + error;
  d_error = error - p_error;
  p_error = error;

  double u = ff + Kp*p_error + Ki*i_error_temp + Kd*d_error;

  // prevent integrator windup and cap the min/max controller output
  if (u_min < u && u < u_max)
    i_error = i_error_temp;
  else if (u < u_min)
    u = u_min;
  else if (u > u_max)
    u = u_max;

  return u;
}

/// @brief Clears the p, i, and d errors that build up during each loop of the controller
void PID::pid_clear(void)
{
  p_error = 0;
  i_error = 0;
  d_error = 0;
}

/// @brief Initializes a desired number of PID Controller objects with the specified configs
/// @param num - number of PID controller objects to create
/// @param Kp_vec - vector containing the Proportional gains for each of the PID controller objects
/// @param Ki_vec - vector containing the Integral gains for each of the PID controller objects
/// @param Kd_vec - vector containing the Derivative gains for each of the PID controller objects
/// @param umin_vec - vector containing the minimum allowable controller output for each of the PID controller objects
/// @param umax_vec - vector containing the maximum allowable controller output for each of the PID controller objects
void MultiPID::multi_pid_init(const uint8_t num, const std::vector<double> Kp_vec, const std::vector<double> Ki_vec, const std::vector<double> Kd_vec, const std::vector<double> umin_vec, const std::vector<double> umax_vec)
{
  num_joints = num;
  pid_vec.clear();
  for (size_t i{0}; i < num_joints; i++)
  {
    PID pid(Kp_vec.at(i), Ki_vec.at(i), Kd_vec.at(i), umin_vec.at(i), umax_vec.at(i));
    pid_vec.push_back(pid);
    joint_ffs.push_back(0);
  }
}

/// @brief Set the desired reference values for each of the PID controller objects
/// @param refs - reference values
void MultiPID::multi_pid_set_refs(const std::vector<double> refs)
{
  joint_refs.clear();
  joint_refs = refs;
}

/// @brief Set the desired feedforward values for each of the PID controller objects
/// @param ffs - feedforward values
void MultiPID::multi_pid_set_ffs(const std::vector<double> ffs)
{
  joint_ffs.clear();
  joint_ffs = ffs;
}

/// @brief Compute one iteration of the control algorithm for each of the PID controller objects
/// @param controller_output [out] - array of size 'num' containing the output of each of the PID controller objects
/// @param joint_actuals - vector containing the current states of each of the joints
void MultiPID::multi_pid_compute_control(double *controller_output, const std::vector<double> joint_actuals)
{
  for (size_t i{0}; i < num_joints; i++)
    controller_output[i] = pid_vec.at(i).pid_compute_control(joint_refs.at(i), joint_ffs.at(i), joint_actuals.at(i));
}

/// @brief Clear the pid errors for each of the PID controller objects
void MultiPID::multi_pid_clear(void)
{
  for (auto& pid : pid_vec)
    pid.pid_clear();
}
