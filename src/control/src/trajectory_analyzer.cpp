// copyright
#include "control/trajectory_analyzer.hpp"

namespace control {
namespace {

double PointDistanceSquare(const TrajectoryPoint &point,
                           const double x, const double y) {
  const double dx = point.path_point().x() - x;
  const double dy = point.path_point().y() - y;
  return dx * dx + dy * dy;
}

TrajectoryPoint TrajectoryPointToPathPoint(const TrajectoryPoint &point) {
  if (point.has_path_point()) {
    return point.path_point();
  } else {
    return PathPoint();
  }
}

}  // namespace

TrajectoryAnalyzer::TrajectoryAnalyzer(
    const Trajectory *planning_published_trajectory) {
  header_time_ = planning_published_trajectory
}

}  // namespace control
