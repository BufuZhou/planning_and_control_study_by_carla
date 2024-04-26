// copyright
#ifndef SRC_VEHICLE_INCLUDE_VEHICLE_VEHICLE_MAIN_HPP_
#define SRC_VEHICLE_INCLUDE_VEHICLE_VEHICLE_MAIN_HPP_

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"
#include "carla_msgs/msg/carla_status.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"

using std::placeholders::_1;
namespace vehicle {

class CarlaVehicleNode : public rclcpp::Node {
 public:
  CarlaVehicleNode();
  // ~vehicle();
  void get_vehicle_state(carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg);
  void set_vehicle_command();
 private:
  double vehicle_accel_;
  double vehicle_velocity_;
  double target_vehicle_accel_;
  double target_steering_angle_;
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr
      ego_vehicle_control_cmd_publisher_;
  carla_msgs::msg::CarlaEgoVehicleControl control_cmd;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr
      ego_vehicle_status_subscriber_;

  size_t count_;
};

}  //  namespace control


#endif