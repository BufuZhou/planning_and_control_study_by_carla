// copyright
#ifndef SRC_PLANNING_INCLUDE_PLANNING_NODE_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_NODE_HPP_
#include "rclcpp/rclcpp.hpp"


namespace planning {

class PlanningNode : public rclcpp::Node {
 public:
  PlanningNode();
 private:
  size_t count_;
};

}  //  namespace planning

#endif  // SRC_PLANNING_INCLUDE_PLANNING_PLANNING_NODE_HPP_

