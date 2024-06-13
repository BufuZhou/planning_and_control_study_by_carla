// copyright
#include <iostream>
#include <algorithm>
#include "control/interpolation_1d.hpp"

namespace control {
namespace common {

const double kDoubleEpsilon = 1e-6;

bool Interpolation1D::Init(const std::vector<std::pair<double, double>>& xy) {
  if (xy.empty()) {
    std::cout << "empty input." << std::endl;
    return false;
  }
  auto data(xy);
  std::sort(data.begin(), data.end());
  Eigen::VectorXd x(data.size());
  Eigen::VectorXd y(data.size());
  for (unsigned i = 0; i < data.size(); ++i) {
    x(i) = data[i].first;
    y(i) = data[i].second;
  }
  x_min_ = data.front().first;
  x_max_ = data.back().first;
  y_start_ = data.front().second;
  y_end_ = data.back().second;

  spline_.reset(new Eigen::Spline<double, 1>(
    Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(
      y.transpose(),
      std::min<int>(x.size() - 1, 3), ScaledValues(x))));
  return true;
}

double Interpolation1D::Interpolate(double x) const {
  if (x < x_min_) {
    return y_start_;
  }
  if (x > x_max_) {
    return y_end_;
  }
  return (*spline_)(ScaledValue(x))(0);
}

double Interpolation1D::ScaledValue(double x) const {
  if (std::fabs(x_max_ - x_min_) < kDoubleEpsilon) {
    return x_min_;
  }
  return (x - x_min_) / (x_max_ - x_min_);
}

Eigen::RowVectorXd Interpolation1D::ScaledValues(
    Eigen::VectorXd const& x_vec) const {
  return x_vec.unaryExpr([this](double x) { return ScaledValue(x); })
      .transpose();
}


}  // namespace common
}  // namespace control
