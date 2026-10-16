#ifndef QWEN_ROBOTNAV_PROGRESS__STABLE_PROGRESS_HPP_
#define QWEN_ROBOTNAV_PROGRESS__STABLE_PROGRESS_HPP_

#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <vector>

namespace qwen_robotnav_progress
{
// Coordinates are Nav2's map pose, not wheel/leg speed or integrated odometry.
// Reset for each FollowPath attempt; Nav2's existing BT owns recovery and retries.
class StableProgress
{
public:
  StableProgress(double radius = .25, double timeout = 12., double window = 2.)
  : radius_(radius), timeout_(timeout), window_(window)
  {
    if (!std::isfinite(radius) || !std::isfinite(timeout) || !std::isfinite(window) ||
      radius <= 0 || window <= 0 || timeout <= window)
    {
      throw std::invalid_argument("Require finite radius > 0 and timeout > window > 0");
    }
  }

  void reset() { samples_.clear(); anchored_ = false; }

  bool check(double time, double x, double y)
  {
    if (!std::isfinite(time) || !std::isfinite(x) || !std::isfinite(y)) {return false;}
    if (!samples_.empty() && time < samples_.back().time) {reset();}
    if (samples_.empty()) {started_ = time;}
    if (samples_.empty() || time > samples_.back().time) {
      samples_.push_back({time, x, y});
    }
    while (samples_.size() > 1 && samples_.front().time < time - window_) {
      samples_.pop_front();
    }
    // Bound memory even if callers provide unexpectedly high pose rates.
    while (samples_.size() > 256) {samples_.pop_front();}
    if (time - started_ < window_) {return true;}
    std::vector<double> xs, ys;
    for (const auto & s : samples_) {xs.push_back(s.x); ys.push_back(s.y);}
    const double mx = median(xs), my = median(ys);
    if (!anchored_ || std::hypot(mx - anchor_x_, my - anchor_y_) >= radius_) {
      anchor_x_ = mx; anchor_y_ = my; progressed_at_ = time; anchored_ = true;
    }
    return time - progressed_at_ <= timeout_;
  }

private:
  struct Sample {double time, x, y;};
  static double median(std::vector<double> & values)
  {
    std::sort(values.begin(), values.end());
    const auto n = values.size();
    return n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2.;
  }
  double radius_, timeout_, window_;
  double started_{0.}, progressed_at_{0.}, anchor_x_{0.}, anchor_y_{0.};
  bool anchored_{false};
  std::deque<Sample> samples_;
};
}  // namespace qwen_robotnav_progress
#endif
