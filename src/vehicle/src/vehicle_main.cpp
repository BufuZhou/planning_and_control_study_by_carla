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
using namespace std::chrono_literals;
using std::placeholders::_1;


namespace vehicle {

CarlaSimulationVehicle::CarlaSimulationVehicle() : Node("Control") {
  int x = 10;
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
}

void CarlaSimulationVehicle::get_vehicle_state() {
  //
}

void CarlaSimulationVehicle::set_vehicle_command() {
  //
}

}  // namespace vehicle

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vehicle::CarlaSimulationVehicle>());
  rclcpp::shutdown();
  return 0;
}
