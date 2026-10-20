#include <limits>
#include <stdexcept>
#include "qwen_robotnav_progress/return_phase.hpp"
using qwen_robotnav_progress::ReturnPhase;
void require(bool ok) {if (!ok) {throw std::runtime_error("RETURN phase regression");}}
int main() {
  ReturnPhase phase;
  using S = ReturnPhase::Stage;
  require(phase.update(.5, 2., .25, .25) == S::Approach);
  require(phase.update(.24, 2., .25, .25) == S::Approach);
  require(phase.update(.12, 2., .25, .25) == S::Approach);
  require(phase.update(.04, 2., .25, .25) == S::Align);
  // Noise around the entry threshold must not switch back to path heading.
  for (double d : {.26, .23, .29, .24, .34}) {
    require(phase.update(d, 1., .25, .25) == S::Align);
  }
  // Real departure, new task and clock/lifecycle reset cannot retain a latch.
  require(phase.update(.36, 1., .25, .25) == S::Approach);
  require(phase.update(.04, 1., .25, .25) == S::Align);
  phase.reset();
  require(phase.update(.30, 1., .25, .25) == S::Approach);
  require(phase.update(.24, .1, .25, .25) == S::Hold);
  // Hysteresis is not a relaxed success tolerance: position outside .25
  // requires approaching again once orientation has settled.
  require(phase.update(.28, .1, .25, .25) == S::Approach);
  require(phase.update(.04, .5, .25, .25) == S::Align);
  require(phase.update(.24, .1, .25, .25) == S::Hold);
  bool rejected = false;
  try {phase.update(std::numeric_limits<double>::quiet_NaN(), .1, .25, .25);}
  catch (const std::invalid_argument &) {rejected = true;}
  require(rejected);
}
