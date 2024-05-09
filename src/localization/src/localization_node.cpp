// Copyright 2016 Open Source Robotics Foundation, Inc.
#include <iostream>
#include <chrono>
#include "localization/localization_node.hpp"
#include "rclcpp/logging.hpp"
// #include "control/lat_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace localization {
LocalizationNode::LocalizationNode() : Node("localization") , count_(0) {
  // // get parameter
  // double mass_fl = 400.0;
  // this->declare_parameter<double>("mass_fl", 0.1);
  // this->get_parameter("mass_fl", mass_fl);
  // RCLCPP_INFO(this->get_logger(), "My parameter value is: %lf", mass_fl);
  // RCLCPP_INFO(this->get_logger(), "My parameter value is: %lf", mass_fl);

  // // get trajectory message
  pose_subscriber_ =
    this->create_subscription<nav_msgs::msg::Odometry>(
        "/carla/ego_vehicle/odometry", 10,
        std::bind(&LocalizationNode::get_location_message, this, _1));

  // // set command to vehicle node
  // ego_vehicle_control_cmd_publisher_ =
  //     this->create_publisher<common_msgs::msg::ControlCommand>(
  //         "/control/control_command", 10);
  // timer_ = this->create_wall_timer(
  //     500ms, std::bind(&ControlNode::send_control_command, this));
}

void LocalizationNode::get_location_message(
    nav_msgs::msg::Odometry::SharedPtr msg) {
  pose_.x = msg->pose.pose.position.x;
  pose_.y = msg->pose.pose.position.y;
  pose_.z = msg->pose.pose.position.z;
  std::cout << "pose.x = " << pose_.x << " "
            << "pose.y = " << pose_.y << " "
            << "pose.z = " << pose_.z << " " << std::endl;
  // 将orientation(四元数)转换为欧拉角(roll, pitch, yaw)
  tf2::Quaternion quat_tf;
  tf2::convert(msg->pose.pose.orientation, quat_tf);
  tf2::Matrix3x3(quat_tf).getRPY(pose_.roll, pose_.pitch, pose_.yaw);
  std::cout << "pose.roll = " << pose_.roll << " "
            << "pose.pitch = " << pose_.pitch << " "
            << "pose.yaw = " << pose_.yaw << " " << std::endl;
  // 速度
  pose_.vel_x = msg->twist.twist.linear.x;
  pose_.vel_y = msg->twist.twist.linear.y;
  pose_.vel_z = msg->twist.twist.linear.z;
  std::cout << "pose.vel_x = " << pose_.vel_x << " "
            << "pose.vel_y = " << pose_.vel_y << " "
            << "pose.vel_z = " << pose_.vel_z << " " << std::endl;
}

// void ControlNode::send_control_command() {
//   LatController temp;
//   control_cmd_.acceleration = 0.5;
//   control_cmd_.steering = temp.get_target_steering_angle();
//   ego_vehicle_control_cmd_publisher_->publish(control_cmd_);
//   RCLCPP_INFO(this->get_logger(), "Publishing %d: %f %f", (count_++),
//               control_cmd_.acceleration, control_cmd_.steering);
// }

}  // namespace localization
