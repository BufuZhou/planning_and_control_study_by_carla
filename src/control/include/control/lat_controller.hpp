// copyright
#ifndef SRC_CONTROL_INCLUDE_CONTROL_LAT_CONTROLLER_HPP_
#define SRC_CONTROL_INCLUDE_CONTROL_LAT_CONTROLLER_HPP_
#include <string>
#include "common_msgs/msg/control_command.hpp"
#include <Eigen/Dense>


namespace control {
class LatController {
 public:
  LatController();
  double get_target_steering_angle();
 private:
  const std::string name_;
  common_msgs::msg::ControlCommand control_cmd_;

  // the following parameters are vehicle physics related
  // control time interval, seconds
  double ts_ = 0.0;
  // front wheel steering stiffness, N/rad
  double cf_ = 0.0;
  // rear wheel steering stiffness, N/rad
  double cr_ = 0.0;
  // distance between the center of the front axle and
  // the center of the rear axle, m
  double wheelbase_ = 0.0;
  // the mass of vehicle, kg
  double mass_ = 0.0;
  // moment of inertia around the z-axis, kg*m^2
  double iz_ = 0.0;
  // vehicle state matrix
  Eigen::MatrixXd matrix_a_;
};
}  // namespace control

#endif  // SRC_CONTROL_INCLUDE_CONTROL_LAT_CONTROLLER_HPP_
