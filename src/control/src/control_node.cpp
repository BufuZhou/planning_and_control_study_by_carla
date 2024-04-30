// Copyright 2016 Open Source Robotics Foundation, Inc.
#include "control/control_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace control {
ControlNode::ControlNode() : Node("control") , count_(0) {
  // set command to vehicle node
  ego_vehicle_control_cmd_publisher_ =
      this->create_publisher<common_msgs::msg::ControlCommand>(
          "/control/vehicle_control_cmd", 10);
  timer_ = this->create_wall_timer(
      500ms, std::bind(&ControlNode::send_control_command, this));
}

void ControlNode::send_control_command() {
  // control_cmd_.acceleration = 0.5;
}

}  // namespace control
