#include <cmath>
#include <iostream>
#include <limits>
#include "qwen_robotnav_progress/stable_progress.hpp"
using qwen_robotnav_progress::StableProgress;
void require(bool value, const char * message) {
  if (!value) {throw std::runtime_error(message);}
}
int main() {
  // Stationary SLAM jitter + rare outliers must not reset progress indefinitely.
  StableProgress stationary;
  double stalled_at = 0.;
  for (int i = 0; i < 300; ++i) {
    double noise = .12 * std::sin(i * 1.7);
    const bool ok = stationary.check(i * .1, i % 27 == 0 ? 1. : noise, -noise);
    if (!ok && !stalled_at) {stalled_at = i * .1;}
  }
  require(stalled_at >= 14. && stalled_at < 16., "Noise hid a stall");
  // Normal driving, curved driving, and an initial turn must not false-trigger.
  for (double speed : {.05, .15, .2}) {
    StableProgress moving;
    for (int i = 0; i < 600; ++i) {
      const double t = i * .1, travelled = speed * std::max(0., t - 7.);
      require(moving.check(t, travelled + .04 * std::sin(i), .15 * std::sin(travelled)),
        "Driving or initial turning falsely stalled");
    }
  }
  stationary.reset();
  require(stationary.check(100., 0., 0.), "Reset did not clear failure");
  require(stationary.check(1., 0., 0.), "Clock reset did not clear old epoch");
  require(!stationary.check(2., std::numeric_limits<double>::quiet_NaN(), 0.),
    "Nonfinite localization was accepted");
  bool rejected = false;
  try {StableProgress invalid(.25, 1., 2.);} catch (const std::invalid_argument &) {rejected = true;}
  require(rejected, "Invalid timing accepted");
  std::cout << "Stable progress scenarios passed\n";
}
