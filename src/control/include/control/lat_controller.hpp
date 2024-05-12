// copyright
#ifndef SRC_CONTROL_INCLUDE_CONTROL_LAT_CONTROLLER_HPP_
#define SRC_CONTROL_INCLUDE_CONTROL_LAT_CONTROLLER_HPP_
#include <Eigen/Dense>
#include <string>
#include "common_msgs/msg/control_command.hpp"
#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/trajectory_point.hpp"

namespace control {
class LatController {
 public:
  LatController();
  double get_target_steering_angle();
  // get the dynamic bicycle model parameters
  void loadControlConfig();
  void computeControlCommand(
      const common_msgs::msg::Pose localization,
      const common_msgs::msg::Trajectory planning_trajectory);
  void QueryNearestPointByPosition(const double x, const double y);
  void computeLateralErrors(const double x, const double y, const double theta,
                            const double linear_v, const double angular_v,
                            const double linear_a);
 private:
  const std::string name_;
  common_msgs::msg::ControlCommand control_cmd_;
  void updateStateSpaceModel();
  // solve discrete-time linear quadratic problem
  void solveLqrProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                       const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                       const double tolerance, const uint max_num_iteration,
                       Eigen::MatrixXd *ptr_K);
  // compute feedforward steering angle
  void computeFeedforward();
  double ts_ = 0.0;         // control time interval, seconds
  // vehicle state
  double velocity_;         // vehicle velocity
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
  Eigen::MatrixXd matrix_b_;   // control matrix
  Eigen::MatrixXd matrix_bd_;  // control matrix (discrete-time)
  Eigen::MatrixXd matrix_k_;   // gain matrix
  Eigen::MatrixXd matrix_r_;   // control weighting matrix
  Eigen::MatrixXd matrix_q_;   // state weighting matrix
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

  // planning trajectory
  common_msgs::msg::Trajectory trajectory_;
  // the trajectory point closest to the actual position of the vehicle
  common_msgs::msg::TrajectoryPoint target_trajectory_point_;
};
}  // namespace control

#endif  // SRC_CONTROL_INCLUDE_CONTROL_LAT_CONTROLLER_HPP_
