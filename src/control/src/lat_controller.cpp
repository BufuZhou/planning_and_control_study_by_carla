// copyright
#include "control/lat_controller.hpp"
#include "rclcpp/logging.hpp"
#include <iostream>

namespace control {
LatController::LatController(): name_("LQR-based Lateral Controller") {
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

void LatController::updateStateSpaceModel() {
  matrix_a_ = Eigen::MatrixXd::Zero(4, 4);
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 1) = -(cf_ + cr_) / mass_ / velocity_;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_ / velocity_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_ / velocity_;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_(3, 3) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_ /velocity_;

  matrix_b_ = Eigen::MatrixXd::Zero(4, 1);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;

  // discretization of State Space model
  Eigen::MatrixXd matrix_i = Eigen::MatrixXd::Identity(4, 4);
  matrix_ad_ = (matrix_i + 0.5 * ts_ * matrix_a_) *
               (matrix_i - 0.5 * ts_ * matrix_a_).inverse();
  matrix_bd_ = matrix_b_ * ts_;
}
double LatController::get_target_steering_angle() {
  return control_cmd_.steering;
}

}  // namespace control
