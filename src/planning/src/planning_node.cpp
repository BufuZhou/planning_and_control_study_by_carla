// copyright
#include "planning/planning_node.hpp"

namespace planning {
PlanningNode::PlanningNode() : Node("planning") , count_(0) {
  // get ego vehicle status
  // std::cout << "plannint init" << std::endl;
}
}  // namespace planning
