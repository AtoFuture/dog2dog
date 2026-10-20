#include "nav2_core/controller.hpp"
#include "nav2_core/progress_checker.hpp"
#include "pluginlib/class_loader.hpp"

int main() {
  // Match controller_server's load order. Linking the RPP library into the
  // progress plugin preloads its factory in the wrong class loader.
  pluginlib::ClassLoader<nav2_core::ProgressChecker> progress_loader("nav2_core", "nav2_core::ProgressChecker");
  auto progress = progress_loader.createSharedInstance("qwen_robotnav_progress::StableProgressChecker");
  pluginlib::ClassLoader<nav2_core::Controller> controller_loader("nav2_core", "nav2_core::Controller");
  auto frontier = controller_loader.createSharedInstance(
    "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController");
  auto returning = controller_loader.createSharedInstance("qwen_robotnav_progress::ReturnController");
  return progress && frontier && returning ? 0 : 1;
}
