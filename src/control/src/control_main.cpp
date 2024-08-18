// Copyright
#include <memory>
#include "glog/logging.h"
#include "control/control_node.hpp"

int main(int argc, char * argv[]) {
  // init glog
  google::InitGoogleLogging(argv[0]);
  LOG(ERROR) << "Hello, World!";
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control::ControlNode>());
  rclcpp::shutdown();
  return 0;
}
