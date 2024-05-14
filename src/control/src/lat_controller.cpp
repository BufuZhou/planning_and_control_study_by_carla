// copyright
#include <iostream>
#include <algorithm>
#include "math.h"
#include "rclcpp/logging.hpp"
#include "common_msgs/msg/pose.hpp"
#include "control/lat_controller.hpp"

namespace control {
LatController::LatController() : name_("LQR-based Lateral Controller") {
  std::cout << "lateral controller: " << name_ << std::endl;
}

void LatController::loadControlConfig() {
  // vehicle parameters
  wheelbase_ =  2.85;
  cf_ = 155494.663;
  cr_ = 155494.663;
  double mass_fl = 600.0;
  double mass_fr = 600.0;
  double mass_rl = 600.0;
  double mass_rr = 600.0;
  double mass_front = mass_fl + mass_fr;
  double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;
  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  // lqr solver paramters
  lqr_eps_ = 0.01;
  lqr_max_iteration_ = 150;
  matrix_q_ = Eigen::MatrixXd::Zero(4, 4);
  matrix_q_ << 0.05, 0.0, 0.0, 0.0,
               0.0,  0.0, 0.0, 0.0,
               0.0,  0.0, 1.0, 0.0,
               0.0,  0.0, 0.0, 0.0;
  matrix_r_ = Eigen::MatrixXd::Identity(1, 1);
}

void LatController::computeLateralErrors(
    const double x, const double y, const double theta,
    common_msgs::msg::Trajectory trajectory) {
  // the trajectory point closest to the actual position of the vehicle
  common_msgs::msg::TrajectoryPoint target_point;

  target_point = QueryNearestPointByPosition(x, y, trajectory);
  double dx = target_point.x - x;
  double dy = target_point.y - y;
  std::cout << "reference heading: " << target_point.theta
            << std::endl;
  std::cout << "ego vehicle heading: " << theta << std::endl;

  double cos_ego_heading = std::cos(target_point.theta);
  double sin_ego_heading = std::sin(target_point.theta);
  lateral_error_ = dy * cos_ego_heading - dx * sin_ego_heading;
  heading_error_ = target_point.theta - std::atan2(dy, dx);

  // todo: how to calculate lateral_error_dot and heading_error_dot
  // lateral_error_dot_ = linear_v * std::sin(heading_error_);
  // heading_error_dot_ = angular_v - target_trajectory_point_.kappa;
  lateral_error_dot_ = 0.0;
  heading_error_ = 0.0;
}

void LatController::updateStateSpaceModel(double vx) {
  vx = std::min(std::abs(vx), 0.01);
  matrix_a_ = Eigen::MatrixXd::Zero(4, 4);
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 1) = -(cf_ + cr_) / mass_ / vx;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_ / vx;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_ / vx;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_(3, 3) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_ /vx;

  matrix_b_ = Eigen::MatrixXd::Zero(4, 1);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;

  // discretization of State Space model
  Eigen::MatrixXd matrix_i = Eigen::MatrixXd::Identity(4, 4);
  matrix_ad_ = (matrix_i + 0.5 * ts_ * matrix_a_) *
               (matrix_i - 0.5 * ts_ * matrix_a_).inverse();
  matrix_bd_ = matrix_b_ * ts_;

  // update state matrix
  matrix_state_ = Eigen::MatrixXd::Zero(4, 1);
  matrix_state_(0, 0) = 0.1;
  matrix_state_(1, 0) = 0.1;
  matrix_state_(2, 0) = 0.1;
  matrix_state_(3, 0) = 0.1;
}

void LatController::solveLqrProblem(
    const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
    const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R, const double tolerance,
    const uint max_num_iteration, Eigen::MatrixXd *ptr_K) {
  if (A.rows() != A.cols() ||
      B.rows() != A.rows() ||
      Q.rows() != Q.cols() ||
      Q.rows() != A.rows() ||
      R.rows() != R.cols() ||
      R.rows() != B.cols()) {
    std::cout << "One or more matrices have incompatible dimensions.";
    return;
  }

  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd BT = B.transpose();
  // Solves a discrete-time Algebraic Riccati equation (DARE)
  // Calculate Matrix Difference Riccati Equation, initialize P and Q
  Eigen::MatrixXd P = Q;
  uint num_iteration = 0;
  double diff = 0.0;
  while (num_iteration++ < max_num_iteration) {
    Eigen::MatrixXd P_next = AT * P * A -
        AT * P * B * (R + BT * P * B).inverse() * BT * P * A +
        Q;
    // check the difference between P and P_next
    diff = fabs((P_next - P).maxCoeff());
    P = P_next;

    if (diff < tolerance) {
      break;
    }
  }

  if (num_iteration >= max_num_iteration) {
    std::cout << "lqr_not_convergence, last_diff_is:" << diff;
  } else {
    std::cout << "Number of iterations until convergence: " << num_iteration
              << ", max difference: " << diff;
  }
  *ptr_K = (R + BT * P * B).inverse() * BT * P * A;
}

void LatController::computeControlCommand(
    const common_msgs::msg::Pose localization,
    const common_msgs::msg::Trajectory planning_trajectory) {
  // the trajectory point closest to the actual position of the vehicle
  common_msgs::msg::TrajectoryPoint target_point;
  loadControlConfig();
  computeLateralErrors(localization.x, localization.y, localization.yaw,
                       planning_trajectory);

  // Calculate the speed of the vehicle along the vehicle's x-axis
  double vx = localization.vel_y * std::cos(localization.yaw) +
              localization.vel_x * std::sin(localization.yaw);
  updateStateSpaceModel(vx);
  solveLqrProblem(matrix_ad_, matrix_bd_, matrix_q_, matrix_r_,
                  lqr_eps_, lqr_max_iteration_, &matrix_k_);
  // feedback = - K * state
  steer_angle_feedback_ = -(matrix_k_ * matrix_state_)(0, 0);
  computeFeedforward(vx);
  steering_angle_command_ = steer_angle_feedback_ + steering_angle_feedforward_;
  std::cout << "steering angle command: " << steering_angle_command_
            << std::endl;
}

void LatController::computeFeedforward(double vx) {
  // cf_ is the sum of lateral stiffness of two front wheels
  // cr_ is the sum of lateral stiffness of two rear wheels
  double kv =
      lr_ * mass_ / cf_ / wheelbase_ - lf_ * mass_ / cr_ / wheelbase_;

  double ref_curvature = 0.002;
  // then change it from rad to %
  steering_angle_feedforward_ =
      wheelbase_ * ref_curvature + kv * vx * vx * ref_curvature -
      matrix_k_(0, 2) *
          (lr_ * ref_curvature - lf_ * mass_ * vx * vx *
                                     ref_curvature / cr_ / wheelbase_);
}

common_msgs::msg::TrajectoryPoint LatController::QueryNearestPointByPosition(
    const double x, const double y,
    common_msgs::msg::Trajectory planning_trajectory) {
  auto trajectory = planning_trajectory.trajectory;
  auto func_distance_square = [](const common_msgs::msg::TrajectoryPoint &point,
                                 const double x, const double y) {
    double dx = point.x - x;
    double dy = point.y - y;
    return dx * dx + dy * dy;
  };

  double d_min =
      func_distance_square(trajectory.front(), x, y);
  size_t index_min = 0;

  for (size_t i = 1; i < trajectory.size(); ++i) {
    double d_temp = func_distance_square(trajectory[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  return trajectory[index_min];
}

double LatController::get_steering_angle_command() {
  return steering_angle_command_;
}

}  // namespace control
