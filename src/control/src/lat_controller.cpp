// copyright
#include "control/lat_controller.hpp"
#include "rclcpp/logging.hpp"
#include <iostream>

namespace control {
LatController::LatController(): name_("LQR-based Lateral Controller") {
  std::cout << "lateral controller: " << name_ << std::endl;
  control_cmd_.steering = 0.02;
}

double LatController::get_target_steering_angle() {
  return control_cmd_.steering;
}

}  // namespace control
