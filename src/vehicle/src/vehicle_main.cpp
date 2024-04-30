// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "vehicle/vehicle_main.hpp"
#include "rclcpp/logging.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


namespace vehicle {

CarlaVehicleNode::CarlaVehicleNode() : Node("Control") , count_(0) {
  // get ego vehicle status
  ego_vehicle_status_subscriber_ =
      this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>(
          "/carla/ego_vehicle/vehicle_status", 10,
          std::bind(&CarlaVehicleNode::get_vehicle_state, this, _1));

  // get control command
  control_command_subscriber_ =
    this->create_subscription<common_msgs::msg::ControlCommand>(
        "/control/control_command", 10,
        std::bind(&CarlaVehicleNode::get_control_command, this, _1));

  // set command to carla vehicle
  ego_vehicle_control_cmd_publisher_ =
      this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
          "/carla/ego_vehicle/vehicle_control_cmd", 10);
  timer_ = this->create_wall_timer(
      500ms, std::bind(&CarlaVehicleNode::send_vehicle_command, this));
}

void CarlaVehicleNode::get_vehicle_state(
    carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg) {
  //
  vehicle_velocity_ = msg->velocity;
  vehicle_accel_ = msg->acceleration.linear.x;
  RCLCPP_INFO(this->get_logger(), "velocity: '%f'", vehicle_velocity_);
  RCLCPP_INFO(this->get_logger(), "vehicle_accel: '%f'", vehicle_accel_);
}

void CarlaVehicleNode::get_control_command(
  common_msgs::msg::ControlCommand::SharedPtr msg) {
  control_command_.acceleration = msg->acceleration;
  control_command_.steering = msg->steering;
}

void CarlaVehicleNode::send_vehicle_command() {
  carla_vehicle_command_.header.stamp = this->now();
  // control_cmd_.header.frame_id
  carla_vehicle_command_.steer = control_command_.steering;
  carla_vehicle_command_.throttle = control_command_.acceleration;
  carla_vehicle_command_.gear = 1;
  carla_vehicle_command_.reverse = false;
  carla_vehicle_command_.manual_gear_shift = false;
  ego_vehicle_control_cmd_publisher_->publish(carla_vehicle_command_);
  RCLCPP_INFO(this->get_logger(), "Publishing %d: '%f'", (count_++),
              carla_vehicle_command_.throttle);
}



}  // namespace vehicle
static const rclcpp::Logger LOGGER = rclcpp::get_logger("carla vehicle");
int main(int argc, char* argv[]) {
  RCLCPP_INFO(LOGGER, "initializa vehicle node");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vehicle::CarlaVehicleNode>());
  rclcpp::shutdown();
  return 0;
}
