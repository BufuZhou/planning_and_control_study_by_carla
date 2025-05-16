// Copyright 2016 Open Source Robotics Foundation, Inc.
#include "control/control_node.hpp"
#include <cmath>
#include <iostream>
#include "common/proto_util.hpp"
#include "glog/logging.h"

namespace control {

// for bind function
using std::placeholders::_1;

// using namespace std::chrono_literals for ms
using std::literals::chrono_literals::operator""""ms;

// controller configure path
constexpr char kControlConfigPath[] =
    "/home/lifanjie/planning_and_control_study_by_carla/src/control/config/"
    "controller_conf.pb.txt";

ControlNode::ControlNode() : Node("control"), count_(0) {
  // load controller configuration
  ::common::ReadProtoFromTextFile(kControlConfigPath, &controller_conf_);
  control_command_.acceleration = 0.43;
  has_subscribed_trajectory_ = false;
  has_subscribed_pose_ = false;

  trajectory_subscriber_ =
      this->create_subscription<common_msgs::msg::Trajectory>(
          "/planning/trajectory", 10,
          std::bind(&ControlNode::get_trajectory, this, _1));

  localization_subscriber_ = this->create_subscription<common_msgs::msg::Pose>(
      "/localization/pose", 10,
      std::bind(&ControlNode::get_localization, this, _1));

  carla_vehicle_control_cmd_publisher_ =
      this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
          "/carla/ego_vehicle/vehicle_control_cmd", 10);

  control_timer_ = this->create_wall_timer(
      10ms, std::bind(&ControlNode::compute_lateral_command, this));

  send_control_command_timer_ = this->create_wall_timer(
      10ms, std::bind(&ControlNode::send_vehicle_command, this));
}

void ControlNode::get_trajectory(common_msgs::msg::Trajectory::SharedPtr msg) {
  has_subscribed_trajectory_ = true;
  trajectory_.trajectory = msg->trajectory;
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
    LOG(INFO) << "not get pose......";
    return;
  } else if (has_subscribed_trajectory_ == false) {
    LOG(INFO) << "not get trajectory......";
    return;
  } else {
    // do nothing
  }

  lateral_controller_.ComputeControlCommand(&pose_, &trajectory_);
  double front_steering_angle = lateral_controller_.GetSteeringAngleCommand();
  control_command_.steering = front_steering_angle * 180 / 3.14156 / 70;
}

void ControlNode::send_vehicle_command() {
  carla_vehicle_command_.header.stamp = this->now();
  // control_cmd_.header.frame_id
  carla_vehicle_command_.steer = -control_command_.steering;
  carla_vehicle_command_.throttle = control_command_.acceleration;
  carla_vehicle_command_.gear = 1;
  carla_vehicle_command_.reverse = false;
  carla_vehicle_command_.manual_gear_shift = false;
  carla_vehicle_control_cmd_publisher_->publish(carla_vehicle_command_);
  LOG(INFO) << "acc: " << carla_vehicle_command_.throttle << ", "
            << "steer angle: " << carla_vehicle_command_.steer;
}

}  // namespace control
