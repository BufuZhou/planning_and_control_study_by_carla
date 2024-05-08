// Copyright 2016 Open Source Robotics Foundation, Inc.
#include <iostream>
#include <chrono>
#include "control/control_node.hpp"
#include "rclcpp/logging.hpp"
#include "control/lat_controller.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

namespace control {
ControlNode::ControlNode() : Node("control") , count_(0) {
  // get parameter
  double mass_fl = 400.0;
  this->declare_parameter<double>("mass_fl", 0.1);
  this->get_parameter("mass_fl", mass_fl);
  RCLCPP_INFO(this->get_logger(), "My parameter value is: %lf", mass_fl);
  RCLCPP_INFO(this->get_logger(), "My parameter value is: %lf", mass_fl);

  // get trajectory message
  trajectory_subscriber_ =
    this->create_subscription<common_msgs::msg::Trajectory>(
        "/planning/trajectory", 10,
        std::bind(&ControlNode::get_trajectory, this, _1));

  // set command to vehicle node
  ego_vehicle_control_cmd_publisher_ =
      this->create_publisher<common_msgs::msg::ControlCommand>(
          "/control/control_command", 10);
  timer_ = this->create_wall_timer(
      500ms, std::bind(&ControlNode::send_control_command, this));
}

void ControlNode::get_trajectory(common_msgs::msg::Trajectory::SharedPtr msg) {
  trajectory_.trajectory = msg->trajectory;
  std::cout << trajectory_.trajectory[0].x << std::endl;
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
