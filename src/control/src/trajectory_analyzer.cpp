// copyright
#include <iostream>
#include <cmath>
#include "control/trajectory_analyzer.hpp"

namespace control {
namespace {

double PointDistanceSquare(const TrajectoryPoint &point,
                           const double x, const double y) {
  const double dx = point.path_point.x - x;
  const double dy = point.path_point.y - y;
  return dx * dx + dy * dy;
}

PathPoint TrajectoryPointToPathPoint(const TrajectoryPoint &point) {
  // if (point.has_path_point()) {
  //   return point.path_point();
  // } else {
  //   return PathPoint();
  // }
  return point.path_point;
}

}  // namespace

TrajectoryAnalyzer::TrajectoryAnalyzer(
    const Trajectory *planning_published_trajectory) {
  auto time = planning_published_trajectory->header.stamp;
  header_time_ =
      static_cast<double>(time.sec) + static_cast<double>(time.nanosec) / 1e9;

  // seq_num_ = planning_published_trajectory->header.frame_id;

  for (unsigned int i = 0; i < planning_published_trajectory->trajectory.size();
       ++i) {
    trajectory_points_.push_back(planning_published_trajectory->trajectory[i]);
  }
}

PathPoint TrajectoryAnalyzer::QueryMatchedPathPoint(const double x,
const double y) const {
  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;

  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }

  size_t index_start = index_min == 0 ? index_min : index_min - 1;
  size_t index_end =
      index_min + 1 == trajectory_points_.size() ? index_min : index_min + 1;

  const double kEpsilon = 0.001;
  if (index_start == index_end ||
      std::fabs(trajectory_points_[index_start].path_point.s -
                trajectory_points_[index_end].path_point.s <= kEpsilon)) {
    return TrajectoryPointToPathPoint(trajectory_points_[index_start]);
  }

  return FindMinDistancePoint(trajectory_points_[index_start],
                              trajectory_points_[index_end], x, y);
}

void TrajectoryAnalyzer::ToTrajectoryFrame(const double x, const double y,
                                           const double theta, const double v,
                                           const PathPoint &ref_point,
                                           double *ptr_s, double *ptr_s_dot,
                                           double *ptr_d,
                                           double *ptr_d_dot) const {
  double dx = x - ref_point.x;
  double dy = y - ref_point.y;

  double cos_ref_theta = std::cos(ref_point.theta);
  double sin_ref_theta = std::sin(ref_point.theta);

  // the sin of diff angle between vector (cos_ref_theta, sin_ref_theta) and
  // (dx, dy)
  double cross_rd_nd = cos_ref_theta * dy - sin_ref_theta * dx;
  *ptr_d = cross_rd_nd;

  // the cos of diff angle between vector (cos_ref_theta, sin_ref_theta) and
  // (dx, dy)
  double dot_rd_nd = dx * cos_ref_theta + dy * sin_ref_theta;
  *ptr_s = ref_point.s + dot_rd_nd;

  double delta_theta = theta - ref_point.theta;
  double cos_delta_theta = std::cos(delta_theta);
  double sin_delta_theta = std::sin(delta_theta);

  *ptr_d_dot = v * sin_delta_theta;

  double one_minus_kappa_r_d = 1 - ref_point.kappa * (*ptr_d);
  if (one_minus_kappa_r_d <= 0.0) {
    std::cout << "TrajectoryAnalyzer::ToTrajectoryFrame "
              "found fatal reference and actual difference. "
              "Control output might be unstable:"
           << " ref_point.kappa:" << ref_point.kappa
           << " ref_point.x:" << ref_point.x
           << " ref_point.y:" << ref_point.y << " car x:" << x
           << " car y:" << y << " *ptr_d:" << *ptr_d
           << " one_minus_kappa_r_d:" << one_minus_kappa_r_d << std::endl;
    // currently set to a small value to avoid control crash.
    one_minus_kappa_r_d = 0.01;
  }
  *ptr_s_dot = v * cos_delta_theta / one_minus_kappa_r_d;
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByAbsoluteTime(
    const double t) const {
  return QueryNearestPointByRelativeTime(t - header_time_);
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByRelativeTime(
  const double t) const {
  auto func_comp = [](const TrajectoryPoint& point,
                      const double relative_time) {
    return point.relative_time < relative_time;
  };

  auto it_low = std::lower_bound(trajectory_points_.begin(),
                                 trajectory_points_.end(), t, func_comp);
  if (it_low == trajectory_points_.begin()) {
    return trajectory_points_.front();
  }

  if (it_low == trajectory_points_.end()) {
    return trajectory_points_.back();
  }

  auto it_lower = it_low - 1;
  if (it_low->relative_time - t < t - it_lower->relative_time) {
    return *it_low;
  }
  return *it_lower;
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByPosition(
    const double x, const double y) const {
  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;

  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  return trajectory_points_[index_min];
}

const std::vector<TrajectoryPoint> &TrajectoryAnalyzer::trajectory_points()
    const {
  return trajectory_points_;
}

PathPoint TrajectoryAnalyzer::FindMinDistancePoint(const TrajectoryPoint &p0,
                                                   const TrajectoryPoint &p1,
                                                   const double x,
                                                   const double y) const {
  // given the fact that the discretized trajectory is dense enough,
  // we assume linear trajectory between consecutive trajectory points.
  // auto dist_square = [&p0, &p1, &x, &y](const double s) {
  //   double px = math::lerp(p0.path_point().x(), p0.path_point().s(),
  //                          p1.path_point().x(), p1.path_point().s(), s);
  //   double py = math::lerp(p0.path_point().y(), p0.path_point().s(),
  //                          p1.path_point().y(), p1.path_point().s(), s);
  //   double dx = px - x;
  //   double dy = py - y;
  //   return dx * dx + dy * dy;
  // };

  PathPoint p = p0.path_point;
  // double s = math::GoldenSectionSearch(dist_square, p0.path_point().s(),
  //                                      p1.path_point().s());
  // p.set_s(s);
  // p.set_x(math::lerp(p0.path_point().x(), p0.path_point().s(),
  //                    p1.path_point().x(), p1.path_point().s(), s));
  // p.set_y(math::lerp(p0.path_point().y(), p0.path_point().s(),
  //                    p1.path_point().y(), p1.path_point().s(), s));
  // p.set_theta(math::slerp(p0.path_point().theta(), p0.path_point().s(),
  //                         p1.path_point().theta(), p1.path_point().s(), s));
  // // approximate the curvature at the intermediate point
  // p.set_kappa(math::lerp(p0.path_point().kappa(), p0.path_point().s(),
  //                        p1.path_point().kappa(), p1.path_point().s(), s));
  return p;
}

}  // namespace control
