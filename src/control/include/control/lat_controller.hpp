// Copyright 2024
#ifndef SRC_CONTROL_INCLUDE_CONTROL_LAT_CONTROLLER_HPP_
#define SRC_CONTROL_INCLUDE_CONTROL_LAT_CONTROLLER_HPP_
#include <Eigen/Dense>
#include <string>
#include <fstream>
#include <memory>
#include "common_msgs/msg/control_command.hpp"
#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/trajectory_point.hpp"
#include "common_msgs/msg/lateral_control_debug.hpp"
#include "control/digital_filter.hpp"
#include "control/mean_filter.hpp"
#include "control/interpolation_1d.hpp"
#include "common_msgs/msg/pose.hpp"

namespace control {

using common_msgs::msg::TrajectoryPoint;
using common_msgs::msg::Pose;
using common_msgs::msg::Trajectory;
using common_msgs::msg::LateralControlDebug;

class LatController {
 public:
  LatController();
  ~LatController();
  double GetSteeringAngleCommand();

  void ComputeControlCommand(const Pose *localization,
                             const Trajectory *planning_trajectory);

 private:
  // lateral controller name
  const std::string name_;
  // for logging purpose
  std::ofstream steer_log_file_;
  // get the dynamic bicycle model parameters
  bool flags_enable_csv_debug;
  void CloseLogFile();
  void LoadControlConfig();
  void ProcessLogs(const LateralControlDebug *debug);
  void LogInitParameters();
  void InitializeFilters();
  // get target trajectory point
  TrajectoryPoint QueryNearestPointByPosition(
    const double x, const double y,
    const Trajectory *trajectory);
  // compute leteral control error
  void ComputeLateralErrors(
      const double x, const double y, const double theta,
      const Trajectory *trajectory);
  // lqr state space model based by vehicle dynamics
  void updateStateSpaceModel(double velocity);
  // solve discrete-time linear quadratic problem
  void solveLqrProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                       const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                       const double tolerance, const uint max_num_iteration,
                       Eigen::MatrixXd *ptr_K);
  // compute feedforward steering angle
  void computeFeedforward(double vx);
  bool Init();
  void LoadLatGainScheduler();
  void Stop();
  std::string Name() const;

  double ts_ = 0.0;         // control time interval, seconds
  // the following parameters are vehicle physics related
  double mass_ = 0.0;       // the mass of vehicle, kg
  double wheelbase_ = 0.0;  // distance between front axle and rear axle, m
  double lf_;               // distance from front wheel center to C.G.
  double lr_;               // distance from rear wheel center to C.G.
  double cf_ = 0.0;         // front wheel steering stiffness, N/rad
  double cr_ = 0.0;         // rear wheel steering stiffness, N/rad
  double iz_ = 0.0;         // moment of inertia around the z-axis, kg*m^2

  // parameters for lqr solver
  double lqr_eps_;             // iteration maximum error
  double lqr_max_iteration_;   // maximum number of iterations
  Eigen::MatrixXd matrix_a_;   // vehicle state matrix
  Eigen::MatrixXd matrix_ad_;  // vehicle state matrix (discrete-time
  Eigen::MatrixXd matrix_adc_;
  Eigen::MatrixXd matrix_a_coeff_;

  Eigen::MatrixXd matrix_b_;   // control matrix
  Eigen::MatrixXd matrix_bd_;  // control matrix (discrete-time)
  Eigen::MatrixXd matrix_bdc_;
  Eigen::MatrixXd matrix_k_;   // gain matrix
  Eigen::MatrixXd matrix_r_;   // control weighting matrix
  Eigen::MatrixXd matrix_q_;   // state weighting matrix
  Eigen::MatrixXd matrix_q_updated_;
  Eigen::MatrixXd matrix_a_coeff;  // vehicle state matrix coefficients
  Eigen::MatrixXd matrix_state_;   // 4 by 1 matrix; state matrix
  // control state
  double lateral_error_;
  double lateral_error_dot_;
  double heading_error_;
  double heading_error_dot_;

  // feedback steering angle
  double steer_angle_feedback_;
  // feedforward steering angle
  double steering_angle_feedforward_;
  // steering angle command
  double steering_angle_command_;

  double max_front_wheel_steer_angle_;  // max front wheel steer angle
  double max_lat_acc_;
  double min_speed_protection_;
  double cutoff_freq_;
  common::DigitalFilter digital_filter_;
  common::MeanFilter distance_error_filter_;
  common::MeanFilter heading_error_filter_;
  uint8_t mean_filter_window_size_;
  int basic_state_size_;
  std::unique_ptr<common::Interpolation1D> distance_error_interpolation_;
  std::unique_ptr<common::Interpolation1D> heading_error_interpolation_;
};
}  // namespace control

#endif  // SRC_CONTROL_INCLUDE_CONTROL_LAT_CONTROLLER_HPP_
