// Copyright 2016 Open Source Robotics Foundation, Inc.
#ifndef SRC_CONTROL_INCLUDE_CONTROL_NODE_HPP_
#define SRC_CONTROL_INCLUDE_CONTROL_NODE_HPP_
// #include "control_cmd.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "common_msgs/msg/control_command.hpp"
#include "memory"
#include "common_msgs/msg/trajectory.hpp"

namespace control {
class ControlNode : public rclcpp::Node {
 public:
  ControlNode();
  void send_control_command();
  void get_trajectory(common_msgs::msg::Trajectory::SharedPtr msg);
 private:
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<common_msgs::msg::ControlCommand>::SharedPtr
    ego_vehicle_control_cmd_publisher_;
  rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr
    trajectory_subscriber_;
  common_msgs::msg::ControlCommand control_cmd_;
  common_msgs::msg::Trajectory trajectory_;
};
}  // namespace control
#endif  // SRC_CONTROL_INCLUDE_CONTROL_CONTROL_NODE_HPP_

