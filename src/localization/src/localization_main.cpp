// Copyright 2016 Open Source Robotics Foundation, Inc.
#include "localization/localization_node.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<localization::LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
