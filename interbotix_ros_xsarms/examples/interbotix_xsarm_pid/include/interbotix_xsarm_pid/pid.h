#ifndef _PID_H_
#define _PID_H_

#include <vector>
#include <cstdlib>
#include <cstdint>

/// Class for a generic PID Controller object - controls a single joint
class PID
{
public:
  /// @brief Constructor for the PID class
  explicit PID(const double Kp = 1.0, const double Ki = 0, const double Kd = 0, const double u_min = -3.14, const double u_max = 3.14);

  /// @brief Compute one iteration of the feedback controller
  /// @param ref - value to track
  /// @param ff - feedforward term
  /// @param actual - observed value
  double pid_compute_control(const double ref, const double ff, const double actual);

  /// @brief Clears the p, i, and d errors that build up during each loop of the controller
  void pid_clear(void);

private:
  double Kp;              // Proportional gain
  double Ki;              // Integral gain
  double Kd;              // Derivative gain
  double u_min;           // Minimum allowable controller output
  double u_max;           // Maximum allowable controller output
  double p_error;         // Proportional error
  double i_error;         // Integral error
  double d_error;         // Derivative error
};

/// Class to manage multiple PID controller objects at the same time - useful for managing multiple joints at one time
class MultiPID
{
public:
  /// @brief Initializes a desired number of PID Controller objects with the specified configs
  /// @param num - number of PID controller objects to create
  /// @param Kp_vec - vector containing the Proportional gains for each of the PID controller objects
  /// @param Ki_vec - vector containing the Integral gains for each of the PID controller objects
  /// @param Kd_vec - vector containing the Derivative gains for each of the PID controller objects
  /// @param umin_vec - vector containing the minimum allowable controller output for each of the PID controller objects
  /// @param umax_vec - vector containing the maximum allowable controller output for each of the PID controller objects
  void multi_pid_init(const uint8_t num, const std::vector<double> Kp_vec, const std::vector<double> Ki_vec, const std::vector<double> Kd_vec, const std::vector<double> umin_vec, const std::vector<double> umax_vec);

  /// @brief Set the desired reference values for each of the PID controller objects
  /// @param refs - reference values
  void multi_pid_set_refs(const std::vector<double> refs);

  /// @brief Set the desired feedforward values for each of the PID controller objects
  /// @param ffs - feedforward values
  void multi_pid_set_ffs(const std::vector<double> ffs);

  /// @brief Compute one iteration of the control algorithm for each of the PID controller objects
  /// @param controller_output [out] - array of size 'num' containing the output of each of the PID controller objects
  /// @param joint_actuals - vector containing the current states of each of the joints
  void multi_pid_compute_control(double *controller_output, const std::vector<double> joint_actuals);

  /// @brief Clear the pid errors for each of the PID controller objects
  void multi_pid_clear(void);

private:
  int num_joints;                     // number of joints to control
  std::vector<PID> pid_vec;           // vector of PID controller objects
  std::vector<double> joint_refs;     // vector of desired reference values to track for each of the joints
  std::vector<double> joint_ffs;      // vector of feedforward terms to use in each of the PID controller objects
};

#endif
