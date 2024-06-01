// copyright
#ifndef SRC_PLANNING_INCLUDE_PLANNING_PLANNING_NODE_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_PLANNING_NODE_HPP_
#include <string>
#include <vector>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "common_msgs/msg/trajectory.hpp"
#include "planning/cubic_spline.hpp"


namespace planning {
class TrajectoryPoint {
 public:
  // relative time from beginning of the trajectory
  double relative_time = 0.0;
  // x coordinate
  double x = 0.0;
  // y coordinate
  double y = 0.0;
  // z coordinate
  double z = 0.0;
  // derivative direction on the x-y plane
  double theta = 0.0;
  // curvature on the x-y planning
  double kappa = 0.0;
  // accumulated distance from beginning of the path
  double s = 0.0;
  // linear velocity
  double v = 0.0;
  // linear acceleration
  double a = 0.0;
  // curvature change rate w.r.t. time
  double dkappa = 0.0;
};


class PlanningNode : public rclcpp::Node {
 public:
  PlanningNode();
  bool loadRoadMap();
  bool computePathProfile(
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<double>* headings, std::vector<double>* accumulated_s,
      std::vector<double>* kappas, std::vector<double>* dkappas);

 private:
  void getWayPoints();
  std::string roadmap_path_;
  double target_velocity_;
  std::vector<TrajectoryPoint> trajectory_points_;
  Spline2D* trajectory_smooth_;
  std::vector<float> way_point_x_;
  std::vector<float> way_point_y_;
  size_t count_;
  common_msgs::msg::Trajectory trajectory_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<common_msgs::msg::Trajectory>::SharedPtr
      trajectory_publisher_;
};

}  //  namespace planning

#endif  // SRC_PLANNING_INCLUDE_PLANNING_PLANNING_NODE_HPP_

