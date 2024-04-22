// copyright
#ifndef SRC_VEHICLE_INCLUDE_VEHICLE_VEHICLE_MAIN_HPP_
#define SRC_VEHICLE_INCLUDE_VEHICLE_VEHICLE_MAIN_HPP_

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_status.h"
#include "carla_msgs/msg/carla_status.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
namespace vehicle {

class CarlaSimulationVehicle : public rclcpp::Node {
 public:
  CarlaSimulationVehicle();
  // ~vehicle();
  void get_vehicle_state();
  void set_vehicle_command();
 private:
  double vehicle_accel_;
  double vehicle_velocity_;
  double target_vehicle_accel_;
  double target_steering_angle_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  carla_msgs::msg::CarlaEgoVehicleControl control_cmd;
};

}  //  namespace control


#endif