// Copyright 2016 Open Source Robotics Foundation, Inc.
#ifndef SRC_CONTROL_INCLUDE_CONTROL_NODE_HPP_
#define SRC_CONTROL_INCLUDE_CONTROL_NODE_HPP_
// #include "control_cmd.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "common_msgs/msg/control_command.hpp"
#include "memory"
#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/pose.hpp"
#include "control/lat_controller.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"

namespace control {
class ControlNode : public rclcpp::Node {
 public:
  ControlNode();
  void send_control_command();
  void send_vehicle_command();
  void get_trajectory(common_msgs::msg::Trajectory::SharedPtr msg);
  void get_localization(common_msgs::msg::Pose::SharedPtr msg);
 private:
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<common_msgs::msg::ControlCommand>::SharedPtr
    ego_vehicle_control_cmd_publisher_;
  rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr
    trajectory_subscriber_;
  rclcpp::Subscription<common_msgs::msg::Pose>::SharedPtr
    localization_subscriber_;
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr
      carla_vehicle_control_cmd_publisher_;
  common_msgs::msg::ControlCommand control_command_;
  common_msgs::msg::Trajectory trajectory_;
  common_msgs::msg::Pose pose_;
  LatController lateral_controller_;
  carla_msgs::msg::CarlaEgoVehicleControl carla_vehicle_command_;
};
}  // namespace control
#endif  // SRC_CONTROL_INCLUDE_CONTROL_CONTROL_NODE_HPP_

