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
  odometry_subscriber_ =
    this->create_subscription<nav_msgs::msg::Odometry>(
        "/carla/ego_vehicle/odometry", 10,
        std::bind(&LocalizationNode::get_location_message, this, _1));

  // send pose message
  pose_publisher_ =
      this->create_publisher<common_msgs::msg::Pose>(
          "/localization/pose", 10);
  timer_ = this->create_wall_timer(
      100ms, std::bind(&LocalizationNode::publish_pose_message, this));
}

void LocalizationNode::get_location_message(
    nav_msgs::msg::Odometry::SharedPtr msg) {
  pose_.x = msg->pose.pose.position.x;
  pose_.y = msg->pose.pose.position.y;
  pose_.z = msg->pose.pose.position.z;
  // std::cout << "pose.x = " << pose_.x << " "
  //           << "pose.y = " << pose_.y << " "
  //           << "pose.z = " << pose_.z << " " << std::endl;
  // 将orientation(四元数)转换为欧拉角(roll, pitch, yaw)
  tf2::Quaternion quat_tf;
  tf2::convert(msg->pose.pose.orientation, quat_tf);
  tf2::Matrix3x3(quat_tf).getRPY(pose_.roll, pose_.pitch, pose_.yaw);
  // std::cout << "pose.roll = " << pose_.roll << " "
  //           << "pose.pitch = " << pose_.pitch << " "
  //           << "pose.yaw = " << pose_.yaw << " " << std::endl;
  // 速度
  pose_.vel_x = msg->twist.twist.linear.x;
  pose_.vel_y = msg->twist.twist.linear.y;
  pose_.vel_z = msg->twist.twist.linear.z;
  // std::cout << "pose.vel_x = " << pose_.vel_x << " "
  //           << "pose.vel_y = " << pose_.vel_y << " "
  //           << "pose.vel_z = " << pose_.vel_z << " " << std::endl;
  pose_.roll_rate =  msg->twist.twist.angular.x;
  pose_.pitch_rate =  msg->twist.twist.angular.y;
  pose_.yaw_rate =  msg->twist.twist.angular.z;
}

void LocalizationNode::publish_pose_message() {
  pose_publisher_->publish(pose_);
  RCLCPP_INFO(this->get_logger(), "Publishing %d: %f %f", (count_++),
              pose_.x, pose_.y);
}

}  // namespace localization
