// copyright
#ifndef SRC_CONTROL_INCLUDE_CONTROL_LAT_CONTROLLER_HPP_
#define SRC_CONTROL_INCLUDE_CONTROL_LAT_CONTROLLER_HPP_
#include <string>
#include "common_msgs/msg/control_command.hpp"

namespace control {
class LatController {
 public:
  LatController();
  double get_target_steering_angle();
 private:
  const std::string name_;
  common_msgs::msg::ControlCommand control_cmd_;
};
}  // namespace control

#endif  // SRC_CONTROL_INCLUDE_CONTROL_LAT_CONTROLLER_HPP_
