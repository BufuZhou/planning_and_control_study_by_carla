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
  vehicle_control_publisher_ =
      this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
          "/carla/ego_vehicle/vehicle_control_cmd", 10);

  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  // timer_ = this->create_wall_timer(
  //     500ms, std::bind(&CarlaVehicleNode::timer_callback, this));
  timer_ = this->create_wall_timer(
      500ms, std::bind(&CarlaVehicleNode::set_vehicle_command, this));
}

void CarlaVehicleNode::get_vehicle_state() {
  //
}

void CarlaVehicleNode::set_vehicle_command() {
  control_cmd.header.stamp = this->now();
  // control_cmd.header.frame_id
  control_cmd.steer = 0.0;
  control_cmd.throttle = 1;
  control_cmd.gear = 1;
  control_cmd.reverse = false;
  control_cmd.manual_gear_shift = false;
  vehicle_control_publisher_->publish(control_cmd);
  RCLCPP_INFO(this->get_logger(), "Publishing%d: '%f'", (count_++),
              control_cmd.throttle);
}

}  // namespace vehicle
static const rclcpp::Logger LOGGER = rclcpp::get_logger("carla vehicle");
int main(int argc, char* argv[]) {
  RCLCPP_INFO(LOGGER, "Initializa Node~");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vehicle::CarlaVehicleNode>());
  rclcpp::shutdown();
  return 0;
}
