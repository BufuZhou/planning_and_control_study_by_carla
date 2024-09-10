// copyright
#include "control/mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

#include "common_msgs/msg/pose.hpp"
// #include "control/digital_filter_coefficients.hpp"
#include "rclcpp/logging.hpp"

namespace control {

MPCController::MPCController() : name_("MPC Controller") {
  std::cout << "controller: " << name_ << std::endl;
  std::cout << "Using " << name_ << std::endl;
  LoadControlConfig();
}

MPCController::~MPCController() { }

void MPCController::LoadControlConfig() {
  std::cout << "load mpc controller config..." << std::endl;
  // vehicle parameters
  ts_ = 0.02;
  wheelbase_ = 2.85;
  cf_ = 166208;
  cr_ = 166208;
  double mass_fl = 424.0;
  double mass_fr = 424.0;
  double mass_rl = 424.0;
  double mass_rr = 424.0;
  double mass_front = mass_fl + mass_fr;
  double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;
  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;
  max_lat_acc_ = 5.0;
  min_speed_protection_ = 0.1;
  // mpc solver paramters
  mpc_eps_ = 0.01;
  mpc_max_iteration_ = 3000;

  // 系统矩阵A和离散后的矩阵A
  matrix_a_ = Eigen::MatrixXd::Zero(6, 6);
  matrix_ad_ = Eigen::MatrixXd::Zero(6, 6);

  // 系统矩阵A中的常量项
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_(4, 5) = 1.0;

  // 控制矩阵B和离散后的控制矩阵B(均为常数项)
  matrix_b_ = Eigen::MatrixXd::Zero(6, 1);
  matrix_bd_ = Eigen::MatrixXd::Zero(6, 1);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_b_(5, 0) =  -1.0;
  matrix_bd_ = matrix_b_ * ts_;

  // 常数项矩阵C和离散后的常数项矩阵C
  matrix_c_ = Eigen::MatrixXd::Zero(6, 1);
  matrix_cd_ = Eigen::MatrixXd::Zero(6, 1);

  // 状态矩阵
  matrix_state_ = Eigen::MatrixXd::Zero(6, 1);
  // 反馈矩阵
  matrix_k_ = Eigen::MatrixXd::Zero(1, 6);
  // 控制权重矩阵
  matrix_r_ = Eigen::MatrixXd::Identity(2, 2);
  matrix_r_(0, 0) = 3.25;
  matrix_r_(0, 0) = 1.0;

  // 状态误差权重矩阵
  matrix_q_ = Eigen::MatrixXd::Zero(6, 6);
  matrix_q_(0, 0) = 40.0;
  matrix_q_(2, 2) = 30.0;
  matrix_q_(4, 4) = 70.0;
  matrix_q_(5, 5) = 10.0;

  last_lateral_error_ = 0.0;
  last_heading_erro_ = 0.0;
}

std::string MPCController::Name() const { return name_; }

void MPCController::ComputeControlCommand(
    const common_msgs::msg::Pose *localization,
    const common_msgs::msg::Trajectory *planning_published_trajectory) {
  std::cout << "mpc controller start......" << std::endl;
  auto target_tracking_trajectory = *planning_published_trajectory;

  // the trajectory point closest to the actual position of the vehicle
  common_msgs::msg::TrajectoryPoint target_point;

  vx_ = std::sqrt(localization->vel_x * localization->vel_x +
                        localization->vel_y * localization->vel_y);
  std::cout << "vx = " << vx_ << std::endl;

  // std::cout << planning_trajectory->trajectory.front().x << std::endl;
  ComputeControlErrors(localization->x, localization->y, localization->yaw,
                       localization->yaw_rate, planning_published_trajectory);

  // Calculate the speed of the vehicle along the vehicle's x-axis
  // double vx = localization->vel_y * std::cos(localization->yaw) +
  //             localization->vel_x * std::sin(localization->yaw);


  updateStateSpaceModel(vx_);
  // solveLqrProblem(matrix_ad_, matrix_bd_, matrix_q_, matrix_r_, lqr_eps_,
  //                 lqr_max_iteration_, &matrix_k_);
  // feedback = - K * state
  steer_angle_feedback_ = -(matrix_k_ * matrix_state_)(0, 0);
  computeFeedforward(vx_);
  steering_angle_command_ = steer_angle_feedback_;
  std::cout << "steering angle command: " << steering_angle_command_
            << std::endl;
  // 最大前轮转角为2deg
  const double max_steer_angle = 30.0 * M_PI / 180.0;
  steering_angle_command_ =
      std::clamp(steering_angle_command_, -max_steer_angle, max_steer_angle);
}

TrajectoryPoint MPCController::QueryNearestPointByPosition(
    const double x, const double y,
    const common_msgs::msg::Trajectory *planning_trajectory) {

  auto trajectory = planning_trajectory->trajectory;
  auto func_distance_square = [](const common_msgs::msg::TrajectoryPoint
  &point,
                                 const double x, const double y) {
    double dx = point.path_point.x - x;
    double dy = point.path_point.y - y;
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
  std::cout << "nearest point........." << std::endl;
  std::cout << index_min << std::endl;

  return trajectory[index_min];
}

void MPCController::ComputeControlErrors(
    const double x, const double y, const double theta, const double yaw_rate,
    const common_msgs::msg::Trajectory *trajectory) {
  // the trajectory point closest to the actual position of the vehicle
  common_msgs::msg::TrajectoryPoint matched_point;

  matched_point = QueryNearestPointByPosition(x, y, trajectory);
  double dx = x - matched_point.path_point.x;
  double dy = y - matched_point.path_point.y;

  std::cout << "match point x: " << matched_point.path_point.x << ", "
            << "match point y: " << matched_point.path_point.y << std::endl;
  std::cout << "ego vehicle point x: " << x << ", "
            << "ego vehicle point y: " << y << std::endl;
  std::cout << "target heading: " << matched_point.path_point.theta << ", "
            << "ego vehicle heading: " << theta << std::endl;

  const double cos_target_heading =
      std::cos(matched_point.path_point.theta);
  const double sin_target_heading =
      std::sin(matched_point.path_point.theta);
  lateral_error_ = cos_target_heading * dy - sin_target_heading * dx;

  heading_error_ = theta - matched_point.path_point.theta;

  std::cout << "lateral error: " << lateral_error_ << ", "
            << "heading error: " << heading_error_ << std::endl;

  // todo: how to calculate lateral_error_dot and heading_error_dot
  const double sin_heading_error = std::sin(heading_error_);
  lateral_error_dot_ = vx_ * sin_heading_error;
  // todo
  heading_error_dot_ =
      yaw_rate - matched_point.v * matched_point.path_point.kappa;
  last_lateral_error_ = lateral_error_;
  last_heading_erro_ = heading_error_;
}

void MPCController::updateStateSpaceModel(double vx) {
  vx = std::max(std::abs(vx), 0.01);

  // 系统矩阵A中与车速相关项，
  matrix_a_(1, 1) = -(cf_ + cr_) / mass_ / vx;
  matrix_a_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_ / vx;
  matrix_a_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_ / vx;
  matrix_a_(3, 3) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_ / vx;

  // discretization of State Space model
  Eigen::MatrixXd matrix_i = Eigen::MatrixXd::Identity(6, 6);
  matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
               (matrix_i + ts_ * 0.5 * matrix_a_);

  // constant matrix
  matrix_c_(1, 0) = (lr_ * cr_ - lf_ * cf_) / mass_ / vx - vx;
  matrix_c_(3, 0) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_ / vx;
  matrix_cd_ = matrix_c_ * ref_heading_rate_ * ts_;

  // update state matrix
  matrix_state_(0, 0) = lateral_error_;
  matrix_state_(1, 0) = lateral_error_dot_;
  matrix_state_(2, 0) = heading_error_;
  matrix_state_(3, 0) = heading_error_dot_;
  matrix_state_(4, 0) = station_error_;
  matrix_state_(5, 0) = speed_error_;
}

void MPCController::solveLqrProblem(
    const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
    const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R, const double tolerance,
    const uint max_num_iteration, Eigen::MatrixXd *ptr_K) {
  if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
      Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols()) {
    std::cout << "One or more matrices have incompatible dimensions.";
    return;
  }

  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd BT = B.transpose();
  // Solves a discrete-time Algebraic Riccati equation (DARE)
  // Calculate Matrix Difference Riccati Equation, initialize P and Q
  Eigen::MatrixXd P = Q;
  uint num_iteration = 0;
  double diff = std::numeric_limits<double>::max();
  while (num_iteration++ < max_num_iteration && diff > tolerance) {
    Eigen::MatrixXd P_next =
        AT * P * A - AT * P * B * (R + BT * P * B).inverse() * BT * P * A + Q;
    // check the difference between P and P_next
    diff = fabs((P_next - P).maxCoeff());
    P = P_next;

    if (diff < tolerance) {
      break;
    }
  }

  if (num_iteration >= max_num_iteration) {
    std::cout << "lqr_not_convergence, last_diff_is:" << diff << std::endl;
  } else {
    std::cout << "Number of iterations until convergence: " << num_iteration
              << ", max difference: " << diff << std::endl;
  }
  *ptr_K = (R + BT * P * B).inverse() * BT * P * A;
}

void MPCController::computeFeedforward(double vx) {
  // cf_ is the sum of lateral stiffness of two front wheels
  // cr_ is the sum of lateral stiffness of two rear wheels
  double kv = lr_ * mass_ / cf_ / wheelbase_ - lf_ * mass_ / cr_ / wheelbase_;

  double ref_curvature = 0.002;
  // then change it from rad to %
  steering_angle_feedforward_ =
      wheelbase_ * ref_curvature + kv * vx * vx * ref_curvature -
      matrix_k_(0, 2) *
          (lr_ * ref_curvature -
           lf_ * mass_ * vx * vx * ref_curvature / cr_ / wheelbase_);
}

double MPCController::GetSteeringAngleCommand() {
  return steering_angle_command_;
}

}  // namespace control
