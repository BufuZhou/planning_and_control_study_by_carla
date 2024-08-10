// Copyright 2016 Open Source Robotics Foundation, Inc.
#include <iostream>
#include <cmath>
#include <chrono>
#include "rclcpp/logging.hpp"
#include "control/control_node.hpp"
#include "common/proto_util.hpp"
#include "lat_based_lqr_controller_conf.pb.h"
#include "controller_conf.pb.h"

using std::placeholders::_1;

// using namespace std::chrono_literals;
// for ms
using std::literals::chrono_literals::operator""ms;

namespace control {
ControlNode::ControlNode() : Node("control"), count_(0) {
  control::pb::ControllerConf controller_conf;
  ::common::ReadProtoFromTextFile(
      "/home/lifanjie/planning_and_control_study_by_carla/src/control/config/"
      "controller_conf.pb.txt",
      &controller_conf);
  std::cout << controller_conf.DebugString() << std::endl;
  // std::cout << controller_conf.cf() << std::endl;
  control_command_.acceleration = 0.50;
  has_subscribed_trajectory_ = false;
  has_subscribed_pose_ = false;
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
  // std::cout << "get pose......." << std::endl;
  pose_.x = msg->x;
  pose_.y = msg->y;
  pose_.vel_x = msg->vel_x;
  pose_.vel_y = msg->vel_y;
  pose_.yaw = msg->yaw;
  // std::cout << "pose_.x = " << pose_.x << " "
  //           << "pose_.y = " << pose_.y << " "
  //           << "pose_.z = " << pose_.z << " " << std::endl;
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
  LatController lateral_controller;
  ControlCommand cmd;
  lateral_controller.ComputeControlCommand(&pose_, &trajectory_, &cmd);
  double front_steering_angle = lateral_controller.GetSteeringAngleCommand();
  RCLCPP_INFO(this->get_logger(), "front_steering_angle: %lf",
              front_steering_angle);
  front_steering_angle = fmin(fmax(front_steering_angle, -70), 70);
  control_command_.steering = front_steering_angle * 180 / 3.14156 / 70;
}

void ControlNode::send_vehicle_command() {
  std::cout << "send vehicle command to carla" << std::endl;
  carla_vehicle_command_.header.stamp = this->now();
  // control_cmd_.header.frame_id
  carla_vehicle_command_.steer = control_command_.steering;
  carla_vehicle_command_.throttle = control_command_.acceleration;
  carla_vehicle_command_.gear = 1;
  carla_vehicle_command_.reverse = false;
  carla_vehicle_command_.manual_gear_shift = false;
  carla_vehicle_control_cmd_publisher_->publish(carla_vehicle_command_);
  RCLCPP_INFO(this->get_logger(), "Publishing %d: %f, %f", (count_++),
              carla_vehicle_command_.throttle, carla_vehicle_command_.steer);
}

}  // namespace control
