// Copyright 2016 Open Source Robotics Foundation, Inc.
#include <iostream>
#include <cmath>
#include <chrono>
#include "rclcpp/logging.hpp"
#include "control/control_node.hpp"
#include "common/proto_util.hpp"


namespace control {

// for bind function
using std::placeholders::_1;

// using namespace std::chrono_literals for ms
using std::literals::chrono_literals::operator""ms;

// set controller configure path
// todo: 20240810 just for test
constexpr char kControlConfigPath[] =
    "/home/lifanjie/planning_and_control_study_by_carla/src/control/config/"
    "controller_conf.pb.txt";

ControlNode::ControlNode() : Node("control"), count_(0) {
  // load controller configuration
  ::common::ReadProtoFromTextFile(kControlConfigPath, &controller_conf_);
  // std::cout << controller_conf.cf() << std::endl;
  control_command_.acceleration = 0.43;
  has_subscribed_trajectory_ = false;
  has_subscribed_pose_ = false;
  // get parameter
  RCLCPP_INFO(this->get_logger(), "control node running.");


  // get trajectory message
  trajectory_subscriber_ =
      this->create_subscription<common_msgs::msg::Trajectory>(
          "/planning/trajectory", 10,
          std::bind(&ControlNode::get_trajectory, this, _1));

  // get pose from localizaion
  localization_subscriber_ = this->create_subscription<common_msgs::msg::Pose>(
      "/localization/pose", 10,
      std::bind(&ControlNode::get_localization, this, _1));

  // set command to carla vehicle
  carla_vehicle_control_cmd_publisher_ =
      this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
          "/carla/ego_vehicle/vehicle_control_cmd", 10);

  control_timer_ = this->create_wall_timer(
      20ms, std::bind(&ControlNode::compute_lateral_command, this));

  timer_ = this->create_wall_timer(
      100ms, std::bind(&ControlNode::send_vehicle_command, this));
}

void ControlNode::get_trajectory(common_msgs::msg::Trajectory::SharedPtr msg) {
  has_subscribed_trajectory_ = true;
  // std::cout << "get trajectory....." << std::endl;
  trajectory_.trajectory = msg->trajectory;
  // std::cout << "x = " << trajectory_.trajectory[0].x << std::endl;
  // std::cout << "y = " << trajectory_.trajectory[0].y << std::endl;
}

void ControlNode::get_localization(common_msgs::msg::Pose::SharedPtr msg) {
  has_subscribed_pose_ = true;
  pose_.x = msg->x;
  pose_.y = msg->y;
  pose_.vel_x = msg->vel_x;
  pose_.vel_y = msg->vel_y;
  pose_.yaw = msg->yaw;
}

void ControlNode::compute_lateral_command() {
  if (has_subscribed_pose_ == false) {
    // std::cout << "not get pose......" << std::endl;
    return;
  } else if (has_subscribed_trajectory_ == false) {
    // std::cout << "not get trajectory......" << std::endl;
    return;
  } else {
    // std::cout << "commpute lateral command........." << std::endl;
  }

  // calculate steering angle by lateral controller
  // ControlCommand cmd;
  lateral_controller_.ComputeControlCommand(&pose_, &trajectory_);
  double front_steering_angle = lateral_controller_.GetSteeringAngleCommand();
  // RCLCPP_INFO(this->get_logger(), "front_steering_angle: %lf",
  //             front_steering_angle);
  // front_steering_angle = fmin(fmax(front_steering_angle, -70), 70);
  control_command_.steering = front_steering_angle * 180 / 3.14156 / 70;
}

void ControlNode::send_vehicle_command() {
  std::cout << "send vehicle command to carla" << std::endl;
  carla_vehicle_command_.header.stamp = this->now();
  // control_cmd_.header.frame_id
  carla_vehicle_command_.steer = -control_command_.steering;
  carla_vehicle_command_.throttle = control_command_.acceleration;
  carla_vehicle_command_.gear = 1;
  carla_vehicle_command_.reverse = false;
  carla_vehicle_command_.manual_gear_shift = false;
  carla_vehicle_control_cmd_publisher_->publish(carla_vehicle_command_);
  RCLCPP_INFO(this->get_logger(), "Publishing %d: %f, %f", (count_++),
              carla_vehicle_command_.throttle, carla_vehicle_command_.steer);
}

}  // namespace control
