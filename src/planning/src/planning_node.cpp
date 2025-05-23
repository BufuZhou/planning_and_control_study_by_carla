// copyright
#include <fstream>
#include <vector>
#include <iostream>
#include <chrono>
#include "planning/planning_node.hpp"
#include "common_msgs/msg/trajectory_point.hpp"


using std::literals::chrono_literals::operator""ms;

namespace planning {
PlanningNode::PlanningNode() : Node("planning") , count_(0) {
  // get ego vehicle status
  // std::cout << "plannint init" << std::endl;
  roadmap_path_ =
      "/home/lifanjie/planning_and_control_study_by_carla/src/planning/data/"
      "reference_line_town03_01.txt";
  target_velocity_ = 5.0;
  loadRoadMap();
  trajectory_smooth_ = new Spline2D(way_point_x_, way_point_y_);
  trajectory_publisher_ = this->create_publisher<common_msgs::msg::Trajectory>(
          "/planning/trajectory", 10);
  timer_ = this->create_wall_timer(
           100ms, std::bind(&PlanningNode::trajectory_pub, this));
}

bool PlanningNode::loadRoadMap() {
  // 读取参考线路径
  std::ifstream infile;
  infile.open(roadmap_path_);  // 将文件流对象与文件连接起来
  if (!infile.is_open()) {
    // std::cout << "open file failed." << std::endl;
    return false;
  } else {
    // std::cout << "open file success." << std::endl;
  }
  std::vector<std::pair<double, double>> xy_points;
  std::string s, x, y;
  while (getline(infile, s)) {
    std::stringstream word(s);
    word >> x;
    word >> y;
    double pt_x = std::atof(x.c_str());
    double pt_y = std::atof(y.c_str());
    xy_points.push_back(std::make_pair(pt_x, pt_y));
  }
  infile.close();
  // Construct the reference_line path profile
  std::vector<double> headings, accumulated_s, kappas, dkappas;
  // 根据离散的点组成的路径，生成路网航向角,累计距离，曲率，曲率的导数
  computePathProfile(xy_points, &headings, &accumulated_s, &kappas, &dkappas);

  for (size_t i = 0; i < headings.size(); i++) {
    common_msgs::msg::TrajectoryPoint trajectory_point;
    trajectory_point.path_point.x = xy_points[i].first;
    trajectory_point.path_point.y = xy_points[i].second;
    trajectory_point.path_point.z = 0.0;
    trajectory_point.path_point.theta = headings[i];
    trajectory_point.path_point.kappa = kappas[i];
    trajectory_point.path_point.s = 0.0;
    trajectory_point.path_point.dkappa = 0.0;
    trajectory_point.path_point.ddkappa = 0.0;
    trajectory_point.path_point.lane_id = "none";
    trajectory_point.v = target_velocity_;
    trajectory_point.a = 0.0;
    trajectory_point.relative_time = 0.0;
    trajectory_.trajectory.push_back(trajectory_point);
  }
  return true;
}

void PlanningNode::trajectory_pub() {
  trajectory_.timestamp =
      static_cast<int64_t>(rclcpp::Clock().now().seconds()*1000);
  // std::cout << trajectory_.timestamp << std::endl;
  trajectory_publisher_->publish(trajectory_);
}


bool PlanningNode::computePathProfile(
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<double>* headings, std::vector<double>* accumulated_s,
      std::vector<double>* kappas, std::vector<double>* dkappas) {
  auto xy_points_ = xy_points;
  headings->clear();
  kappas->clear();
  dkappas->clear();

  if (xy_points_.size() < 2) {
    return false;
  }
  std::vector<double> dxs;
  std::vector<double> dys;
  std::vector<double> y_over_s_first_derivatives;
  std::vector<double> x_over_s_first_derivatives;
  std::vector<double> y_over_s_second_derivatives;
  std::vector<double> x_over_s_second_derivatives;

  // Get finite difference approximated dx and dy for heading and kappa
  // calculation
  std::size_t points_size = xy_points_.size();
  for (std::size_t i = 0; i < points_size; ++i) {
    double x_delta = 0.0;
    double y_delta = 0.0;
    if (i == 0) {
      x_delta = (xy_points_[i + 1].first - xy_points_[i].first);
      y_delta = (xy_points_[i + 1].second - xy_points_[i].second);
    } else if (i == points_size - 1) {
      x_delta = (xy_points_[i].first - xy_points_[i - 1].first);
      y_delta = (xy_points_[i].second - xy_points_[i - 1].second);
    } else {
      x_delta = 0.5 * (xy_points_[i + 1].first - xy_points_[i - 1].first);
      y_delta = 0.5 * (xy_points_[i + 1].second - xy_points_[i - 1].second);
    }
    dxs.push_back(x_delta);
    dys.push_back(y_delta);
  }

  // Heading calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    headings->push_back(std::atan2(dys[i], dxs[i]));
  }

  // Get linear interpolated s for dkappa calculation
  double distance = 0.0;
  accumulated_s->push_back(distance);
  double fx = xy_points_[0].first;
  double fy = xy_points_[0].second;
  double nx = 0.0;
  double ny = 0.0;
  for (std::size_t i = 1; i < points_size; ++i) {
    nx = xy_points_[i].first;
    ny = xy_points_[i].second;
    double end_segment_s =
        std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
    accumulated_s->push_back(end_segment_s + distance);
    distance += end_segment_s;
    fx = nx;
    fy = ny;
  }

  // Get finite difference approximated first derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = 0.0;
    double yds = 0.0;
    if (i == 0) {
      xds = (xy_points_[i + 1].first - xy_points_[i].first) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i));
      yds = (xy_points_[i + 1].second - xy_points_[i].second) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      xds = (xy_points_[i].first - xy_points_[i - 1].first) /
            (accumulated_s->at(i) - accumulated_s->at(i - 1));
      yds = (xy_points_[i].second - xy_points_[i - 1].second) /
            (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      xds = (xy_points_[i + 1].first - xy_points_[i - 1].first) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      yds = (xy_points_[i + 1].second - xy_points_[i - 1].second) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_first_derivatives.push_back(xds);
    y_over_s_first_derivatives.push_back(yds);
  }

  // Get finite difference approximated second derivative of y and x
  // respective to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double xdds = 0.0;
    double ydds = 0.0;
    if (i == 0) {
      xdds =
          (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
          (accumulated_s->at(i + 1) - accumulated_s->at(i));
      ydds =
          (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
          (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      xdds =
          (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
          (accumulated_s->at(i) - accumulated_s->at(i - 1));
      ydds =
          (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
          (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      xdds = (x_over_s_first_derivatives[i + 1] -
              x_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      ydds = (y_over_s_first_derivatives[i + 1] -
              y_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_second_derivatives.push_back(xdds);
    y_over_s_second_derivatives.push_back(ydds);
  }

  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = x_over_s_first_derivatives[i];
    double yds = y_over_s_first_derivatives[i];
    double xdds = x_over_s_second_derivatives[i];
    double ydds = y_over_s_second_derivatives[i];
    double kappa =
        (xds * ydds - yds * xdds) /
        (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
    kappas->push_back(kappa);
  }

  // Dkappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double dkappa = 0.0;
    if (i == 0) {
      dkappa = (kappas->at(i + 1) - kappas->at(i)) /
               (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      dkappa = (kappas->at(i) - kappas->at(i - 1)) /
               (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      dkappa = (kappas->at(i + 1) - kappas->at(i - 1)) /
               (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    dkappas->push_back(dkappa);
  }
  return true;
}

}  // namespace planning
