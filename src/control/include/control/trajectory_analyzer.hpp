// copyright
#ifndef SRC_CONTROL_INCLUDE_CONTROL_TRAJECTORY_ANALYZER_HPP_
#define SRC_CONTROL_INCLUDE_CONTROL_TRAJECTORY_ANALYZER_HPP_

#include <vector>

#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/trajectory_point.hpp"

using common_msgs::msg::Trajectory;
using common_msgs::msg::TrajectoryPoint;

namespace control {

class TrajectoryAnalyzer {
 public:
  TrajectoryAnalyzer() = default;
  explicit TrajectoryAnalyzer(const Trajectory *planning_published_trajectory);
  ~TrajectoryAnalyzer() = default;
  unsigned int seq_num() { return seq_num_; }
  TrajectoryPoint QueryNearestPointByAbsoluteTime(const double t) const;
  TrajectoryPoint QueryNearestPointByRelativeTime(const double t) const;
  TrajectoryPoint QueryNearestPointByPosition(const double x,
                                              const double y) const;
  TrajectoryPoint QueryMatchedPathPoint(const double x, const double y) const;
  void ToTrajectoryFrame(const double x, const double y, const double theta,
                         const double v, const TrajectoryPoint &matched_point,
                         double *ptr_s, double *ptr_s_dot, double *ptr_d,
                         double *ptr_d_dot) const;
  const std::vector<TrajectoryPoint> &trajectory_points() const;

 private:
  TrajectoryPoint FindMinDistance(const TrajectoryPoint &p0,
                                  const TrajectoryPoint &p1, const double x,
                                  const double y) const;
  std::vector<TrajectoryPoint> trajectory_points_;
  double header_time_ = 0.0;
  double int seq_num_ = 0;
};

}  // namespace control

#endif  // SRC_CONTROL_INCLUDE_CONTROL_TRAJECTORY_ANALYZER_HPP_
