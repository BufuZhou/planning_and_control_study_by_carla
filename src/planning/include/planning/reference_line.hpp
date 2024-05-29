// copyright
#ifndef SRC_PLANNING_INCLUDE_PLANNING_REFERENCE_LINE_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_REFERENCE_LINE_HPP_
#include <math.h>
#include <iostream>
#include <vector>
#include <utility>

namespace planning {
class ReferenceLine {
 public:
  explicit ReferenceLine(
      const std::vector<std::pair<double, double>>& xy_points);
  ~ReferenceLine() = default;

  bool ComputePathProfile(std::vector<double>* headings,
                          std::vector<double>* accumulated_s,
                          std::vector<double>* kappas,
                          std::vector<double>* dkappas);

 private:
  std::vector<std::pair<double, double>> xy_points_;
};

}  // namespace planning

#endif  // SRC_PLANNING_INCLUDE_PLANNING_REFERENCE_LINE_HPP_
