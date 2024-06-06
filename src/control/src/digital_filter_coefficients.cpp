// copyright
#include <cmath>
#include <vector>
#include "control/digital_filter_coefficients.hpp"

namespace control {
namespace common {
void LpfCoefficients(const double ts, const double cutoff_freq,
                     std::vector<double> *denominators,
                     std::vector<double> *nummerators) {
  denominators->clear();
  nummerators->clear();
  denominators->reserve(3);
  nummerators->reserve(3);

  double wa = 2.0 * M_PI * cutoff_freq;
  double alpha = wa * ts / 2.0;
  double alpha_sqr = alpha * alpha;
  double tmp_term = std::sqrt(2.0) * alpha + alpha_sqr;
  double gain = alpha_sqr / (1.0 + tmp_term);

  denominators->push_back(1.0);
  denominators->push_back(2.0 * (alpha_sqr - 1.0) / (1.0 + tmp_term));
  denominators->push_back((1.0 - std::sqrt(2.0) * alpha + alpha_sqr) /
                          (1.0 + tmp_term));

  nummerators->push_back(gain);
  nummerators->push_back(2.0 * gain);
  nummerators->push_back(gain);

  return;
}
}  // namespace common
}  // namespace control
