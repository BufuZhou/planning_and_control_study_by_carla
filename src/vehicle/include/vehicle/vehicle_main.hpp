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
#include "common_msgs/msg/control_command.hpp"

using std::placeholders::_1;
namespace vehicle {

class CarlaVehicleNode : public rclcpp::Node {
 public:
  CarlaVehicleNode();
  // ~vehicle();
  void get_vehicle_state(carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg);
  void send_vehicle_command();
  void get_control_command(common_msgs::msg::ControlCommand::SharedPtr msg);
 private:
  double vehicle_accel_;
  double vehicle_velocity_;
  double target_vehicle_accel_;
  double target_steering_angle_;
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr
      ego_vehicle_control_cmd_publisher_;
  rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr
      ego_vehicle_status_subscriber_;
  rclcpp::Subscription<common_msgs::msg::ControlCommand>::SharedPtr
      control_command_subscriber_;
  carla_msgs::msg::CarlaEgoVehicleControl carla_vehicle_command_;
  common_msgs::msg::ControlCommand control_command_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

}  //  namespace control


#endif