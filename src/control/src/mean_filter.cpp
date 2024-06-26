// copyright
#include <limits>
#include "control/mean_filter.hpp"

namespace control {
namespace common {

using MF = MeanFilter;
using uint8 = uint_fast8_t;
using TimedValue = std::pair<uint8, double>;

const uint8 kMaxWindowSize = std::numeric_limits<uint8>::max() / 2;

MF::MeanFilter(const uint8 window_size) : window_size_(window_size) {
  initialized_ = true;
}

double MF::GetMin() const {
  if (min_candidates_.size() == 0) {
    return std::numeric_limits<double>::infinity();
  } else {
    return min_candidates_.front().second;
  }
}

double MF::GetMax() const {
  if (max_candidates_.size() == 0) {
    return -std::numeric_limits<double>::infinity();
  } else {
    return max_candidates_.front().second;
  }
}

double MF::Update(const double measurement) {
  ++time_;
  time_ %= 2 * window_size_;
  if (values_.size() == window_size_) {
    RemoveEarliest();
  }
  Insert(measurement);
  if (values_.size() > 2) {
    return (sum_ - GetMin() - GetMax()) / (values_.size() - 2);
  } else {
    return sum_ / values_.size();
  }
}

bool MF::ShouldPopOldestCandidate(const uint8 old_time) const {
  if (old_time < window_size_) {
    return old_time + window_size_ == time_;
  } else if (time_ < window_size_) {
    return old_time == time_ + window_size_;
  } else {
    return false;
  }
}

void MF::RemoveEarliest() {
  double removed = values_.front();
  values_.pop_front();
  sum_ -= removed;
  if (ShouldPopOldestCandidate(min_candidates_.front().first)) {
    min_candidates_.pop_front();
  }
  if (ShouldPopOldestCandidate(max_candidates_.front().first)) {
    max_candidates_.pop_front();
  }
}

void MF::Insert(const double value) {
  values_.push_back(value);
  sum_ += value;
  while (min_candidates_.size() > 0 && min_candidates_.back().second > value) {
    min_candidates_.pop_back();
  }
  min_candidates_.push_back(std::make_pair(time_, value));
  while (max_candidates_.size() > 0 && max_candidates_.back().second < value) {
    max_candidates_.pop_back();
  }
  max_candidates_.push_back(std::make_pair(time_, value));
}

}  // namespace common
}  // namespace control
