#pragma once
#include <cmath>
#include <stdexcept>

namespace qwen_robotnav_progress {
// Goal positions use the same frame throughout a command. Replanning to the
// same endpoint must not reset alignment; a correction/new endpoint must.
class ReturnPhase {
public:
  enum class Stage {Approach, Align, Hold};
  static double entry_tolerance(double xy_tolerance) {return xy_tolerance * .2;}
  static double turn_handoff_tolerance(double xy_tolerance) {return xy_tolerance * .5;}
  void reset() {stage_ = Stage::Approach;}
  Stage update(double distance, double yaw_error, double xy_tolerance, double yaw_tolerance,
    bool approach_requires_turn = false) {
    if (!std::isfinite(distance) || !std::isfinite(yaw_error) ||
      !(xy_tolerance > 0.) || !(yaw_tolerance > 0.)) {
      throw std::invalid_argument("Invalid RETURN geometry or tolerance");
    }
    if (distance > xy_tolerance + .10) {stage_ = Stage::Approach;}
    if (distance <= xy_tolerance && std::abs(yaw_error) <= yaw_tolerance) {
      stage_ = Stage::Hold;
      return stage_;
    }
    // Reserve most of the accepted position radius for the observed rotation
    // displacement, rather than starting a turn on its outer boundary.
    // A lateral endpoint can be inside the accepted position disk while the
    // tight approach radius remains unreachable under corridor centering.
    // If pursuit would rotate again, hand off inside half the actual disk;
    // retain the other half for turn displacement. Straight approaches still
    // use the original tighter entry. Actual success tolerances never change.
    if (stage_ == Stage::Approach &&
      (distance <= entry_tolerance(xy_tolerance) ||
      (approach_requires_turn && distance <= turn_handoff_tolerance(xy_tolerance)))) {
      stage_ = Stage::Align;
    }
    if (stage_ != Stage::Approach) {
      if (std::abs(yaw_error) <= yaw_tolerance) {
        stage_ = distance <= xy_tolerance ? Stage::Hold : Stage::Approach;
      } else {stage_ = Stage::Align;}
    }
    return stage_;
  }
private:
  Stage stage_{Stage::Approach};
};
}  // namespace qwen_robotnav_progress
