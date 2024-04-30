// Copyright 2016 Open Source Robotics Foundation, Inc.
#ifndef SRC_CONTROL_INCLUDE_CONTROL_NODE_HPP_
#define SRC_CONTROL_INCLUDE_CONTROL_NODE_HPP_
// #include "control_cmd.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "common_msgs/msg/control_command.hpp"
#include "memory"

namespace control {
class ControlNode : public rclcpp::Node {
 public:
  ControlNode();
  void send_control_command();
 private:
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<common_msgs::msg::ControlCommand>::SharedPtr
    ego_vehicle_control_cmd_publisher_;
  common_msgs::msg::ControlCommand control_cmd_;
};
}  // namespace control
#endif  // SRC_CONTROL_INCLUDE_CONTROL_CONTROL_NODE_HPP_

