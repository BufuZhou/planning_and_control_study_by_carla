// Copyright 2016 Open Source Robotics Foundation, Inc.
#include "control/control_node.hpp"
#include "rclcpp/logging.hpp"
#include <chrono>
#include "control/lat_controller.hpp"

using namespace std::chrono_literals;

namespace control {
ControlNode::ControlNode() : Node("control") , count_(0) {
  // set command to vehicle node
  ego_vehicle_control_cmd_publisher_ =
      this->create_publisher<common_msgs::msg::ControlCommand>(
          "/control/control_command", 10);
  timer_ = this->create_wall_timer(
      500ms, std::bind(&ControlNode::send_control_command, this));
}

void ControlNode::send_control_command() {
  LatController temp;
  control_cmd_.acceleration = 0.5;
  control_cmd_.steering = temp.get_target_steering_angle();
  ego_vehicle_control_cmd_publisher_->publish(control_cmd_);
  RCLCPP_INFO(this->get_logger(), "Publishing %d: %f %f", (count_++),
              control_cmd_.acceleration, control_cmd_.steering);
}

}  // namespace control
