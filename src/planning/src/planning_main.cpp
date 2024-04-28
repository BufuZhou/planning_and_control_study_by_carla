// Copyright 2016 Open Source Robotics Foundation, Inc.
#include "rclcpp/logging.hpp"
#include "planning/planning_node.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("planning node");
int main(int argc, char* argv[]) {
  RCLCPP_INFO(LOGGER, "initializa planning node");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<planning::PlanningNode>());
  rclcpp::shutdown();
  return 0;
}
