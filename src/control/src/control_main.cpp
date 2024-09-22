// Copyright
#include <memory>
#include "glog/logging.h"
#include "control/control_node.hpp"

int main(int argc, char * argv[]) {
  // init glog
  google::InitGoogleLogging(argv[0]);
  // FLAGS_log_dir = ".";
  FLAGS_log_dir = "/home/lifanjie/glog";
  FLAGS_alsologtostderr = 1;              // 日志同时输出到stderr

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control::ControlNode>());
  rclcpp::shutdown();
  // 关闭glog
  google::ShutdownGoogleLogging();
  return 0;
}
