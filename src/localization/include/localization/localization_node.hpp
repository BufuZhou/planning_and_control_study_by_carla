// Copyright 2016 Open Source Robotics Foundation, Inc.
#ifndef SRC_LOCALIZATION_INCLUDE_LOCALIZATION_LOCALIZATION_NODE_HPP_
#define SRC_LOCALIZATION_INCLUDE_LOCALIZATION_LOCALIZATION_NODE_HPP_
#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp/rclcpp.hpp"
// #include "common_msgs/msg/control_command.hpp"
#include "memory"
#include "common_msgs/msg/pose.hpp"

namespace localization {

// struct pose {
//   double timestamp;
//   double x;
//   double y;
//   double z;
//   double yaw;
//   double pitch;
//   double roll;
//   double vel_x;
//   double vel_y;
//   double vel_z;
//   double acc_x;
//   double acc_y;
//   double acc_z;
// };

class LocalizationNode : public rclcpp::Node {
 public:
  LocalizationNode();
  // void send_control_command();
  void get_location_message(nav_msgs::msg::Odometry::SharedPtr msg);
 private:
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Publisher<common_msgs::msg::ControlCommand>::SharedPtr
  //   ego_vehicle_control_cmd_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    pose_subscriber_;
  common_msgs::msg::Pose pose_;
};
}  // namespace localization
#endif  // SRC_LOCALIZATION_INCLUDE_LOCALIZATION_LOCALIZATION_NODE_HPP_


