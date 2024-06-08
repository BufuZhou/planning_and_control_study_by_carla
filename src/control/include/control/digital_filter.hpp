// copyright
#ifndef SRC_CONTROL_INCLUDE_CONTROL_DIGITAL_FILTER_HPP_
#define SRC_CONTROL_INCLUDE_CONTROL_DIGITAL_FILTER_HPP_
#include <vector>
#include <deque>

namespace control {
namespace common {
class DigitalFilter {
 public:
  DigitalFilter() = default;
  DigitalFilter(const std::vector<double> &denominators,
                const std::vector<double> &numerators);
  ~DigitalFilter() = default;
  double Filter(const double x_insert);
  void set_denominators(const std::vector<double> &denominators);
  void set_numerators(const std::vector<double> &numerators);
  void set_coefficients(const std::vector<double> &denominators,
                        const std::vector<double> &numerators);
  void set_dead_zone(const double dead_zone);
  const std::vector<double> &denominators() const;
  const std::vector<double> &numerators() const;
  double dead_zone() const;

 private:
  double UpdateLast(const double input);
  double Compute(const std::deque<double> &values,
                 const std::vector<double> &coefficients,
                 const std::size_t coeff_start,
                 const std::size_t coeff_end);
  std::deque<double> x_values_;
  std::deque<double> y_values_;
  std::vector<double> denominators_;
  std::vector<double> numerators_;
  double dead_zone_ = 0.0;
  double last_ = 0.0;
};

}  // namespace common
}  // namespace control

#endif  // SRC_CONTROL_INCLUDE_CONTROL_DIGITAL_FILTER_HPP_
