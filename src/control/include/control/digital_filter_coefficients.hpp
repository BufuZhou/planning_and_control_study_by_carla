// copyright
#ifndef SRC_CONTROL_INCLUDE_CONTROL_DIGITAL_FILTER_COEFFICIENTS_HPP_
#define SRC_CONTROL_INCLUDE_CONTROL_DIGITAL_FILTER_COEFFICIENTS_HPP_
#include <vector>
namespace control {
namespace common {
void LpfCoefficients(const double ts, const double cutoff_freq,
                     std::vector<double> *denominators,
                     std::vector<double> *nummerators);
}  // namespace common
}  // namespace control
#endif  // SRC_CONTROL_INCLUDE_CONTROL_DIGITAL_FILTER_COEFFICIENTS_HPP_
