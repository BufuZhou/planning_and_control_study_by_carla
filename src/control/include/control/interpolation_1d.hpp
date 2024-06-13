// copyright
#ifndef SRC_CONTROL_INCLUDE_CONTROL_INTERPOLATION_1D_HPP_
#define SRC_CONTROL_INCLUDE_CONTROL_INTERPOLATION_1D_HPP_
#include <utility>
#include <vector>
#include <memory>
#include "Eigen/Core"
#include "unsupported/Eigen/Splines"

namespace control {
namespace common {

class Interpolation1D {
 public:
  Interpolation1D() = default;
  bool Init(const std::vector<std::pair<double, double>>& xy);
  double Interpolate(double x) const;

 private:
  double ScaledValue(double x) const;
  Eigen::RowVectorXd ScaledValues(Eigen::VectorXd const& x_vec) const;

  double x_min_ = 0.0;
  double x_max_ = 0.0;
  double y_start_ = 0.0;
  double y_end_ = 0.0;

  std::unique_ptr<Eigen::Spline<double, 1>> spline_;
};

}  // namespace common
}  // namespace control

#endif  // SRC_CONTROL_INCLUDE_CONTROL_INTERPOLATION_1D_HPP_
