// Copyright 2016 Open Source Robotics Foundation, Inc.
#include <iostream>
#include <chrono>
#include "control/control_node.hpp"
#include "rclcpp/logging.hpp"



using namespace std::chrono_literals;
using std::placeholders::_1;

namespace control {
ControlNode::ControlNode() : Node("control") , count_(0) {
  //
  control_command_.acceleration = 0.5;
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

  // get pose from localizaion
  localization_subscriber_ =
      this->create_subscription<common_msgs::msg::Trajectory>(
          "/localization/pose", 10,
          std::bind(&ControlNode::get_localization, this, _1));

  // set command to carla vehicle
  carla_vehicle_control_cmd_publisher_ =
      this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
          "/carla/ego_vehicle/vehicle_control_cmd", 10);
  timer_ = this->create_wall_timer(
      500ms, std::bind(&ControlNode::send_vehicle_command, this));

  // set command to vehicle node
  // ego_vehicle_control_cmd_publisher_ =
  //     this->create_publisher<common_msgs::msg::ControlCommand>(
  //         "/control/control_command", 10);
  // timer_ = this->create_wall_timer(
  //     500ms, std::bind(&ControlNode::send_control_command, this));
}

void ControlNode::get_trajectory(common_msgs::msg::Trajectory::SharedPtr msg) {
  trajectory_.trajectory = msg->trajectory;
  std::cout << trajectory_.trajectory[0].x << std::endl;
}

void ControlNode::get_localization(common_msgs::msg::Pose::SharedPtr msg) {
  pose_.x = msg->x;
  pose_.y = msg->y;
  pose_.vel_x = msg->vel_x;
  pose_.vel_y = msg->vel_y;
  pose_.yaw = msg->yaw;
  std::cout << "pose_.x = " << pose_.x << " " << "pose_.y = " << pose_.y << " "
            << "pose_.z = " << pose_.z << " " << std::endl;
}

void ControlNode::send_vehicle_command() {
  // calculate steering angle by lateral controller
  // lateral_controller_.computeControlCommand(pose_, trajectory_);
  // control_command_.steering = lateral_controller_.get_steering_angle_command();

  carla_vehicle_command_.header.stamp = this->now();
  // control_cmd_.header.frame_id
  carla_vehicle_command_.steer = control_command_.steering;
  carla_vehicle_command_.throttle = control_command_.acceleration;
  carla_vehicle_command_.gear = 1;
  carla_vehicle_command_.reverse = false;
  carla_vehicle_command_.manual_gear_shift = false;
  carla_vehicle_control_cmd_publisher_->publish(carla_vehicle_command_);
  RCLCPP_INFO(this->get_logger(), "Publishing %d: '%f'", (count_++),
              carla_vehicle_command_.throttle);
}

void ControlNode::send_control_command() {
  // LatController temp;
  // control_command_.acceleration = 0.5;
  // control_cmd_.steering = temp.get_target_steering_angle();
  // ego_vehicle_control_cmd_publisher_->publish(control_command_);
  // RCLCPP_INFO(this->get_logger(), "Publishing %d: %f %f", (count_++),
  //             control_command_.acceleration, control_command_.steering);
}

}  // namespace control
