// copyright
#ifndef SRC_CONTROL_INCLUDE_CONTROL_MEAN_FILTER_HPP_
#define SRC_CONTROL_INCLUDE_CONTROL_MEAN_FILTER_HPP_

#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

namespace control {
namespace common {

class MeanFilter {
 public:
  explicit MeanFilter(const std::uint_fast8_t window_size);
  MeanFilter() = default;
  ~MeanFilter() = default;
  double Update(const double measurement);

 private:
  void RemoveEarliest();
  void Insert(const double value);
  double GetMin() const;
  double GetMax() const;
  bool ShouldPopOldestCandidate(const std::uint_fast8_t old_time) const;
  std::uint_fast8_t window_size_ = 0;
  double sum_ = 0.0;
  std::uint_fast8_t time_ = 0;
  std::deque<double> values_;
  std::deque<std::pair<std::uint_fast8_t, double>> min_candidates_;
  std::deque<std::pair<std::uint_fast8_t, double>> max_candidates_;
  bool initialized_ = false;
};

}  // namespace common
}  // namespace control

#endif  // SRC_CONTROL_INCLUDE_CONTROL_MEAN_FILTER_HPP_

