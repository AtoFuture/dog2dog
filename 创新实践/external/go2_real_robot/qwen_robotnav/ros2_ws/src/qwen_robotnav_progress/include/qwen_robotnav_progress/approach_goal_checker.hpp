#pragma once
#include "nav2_core/goal_checker.hpp"
#include "qwen_robotnav_progress/return_phase.hpp"

namespace qwen_robotnav_progress {
// RPP's internal final-yaw switch must use the same tighter entry radius;
// the controller server's actual success checker keeps its original .25m.
class ApproachGoalChecker : public nav2_core::GoalChecker {
public:
  explicit ApproachGoalChecker(nav2_core::GoalChecker * checker) : checker_(checker) {}
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {checker_->reset();}
  bool isGoalReached(const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Pose & goal,
    const geometry_msgs::msg::Twist & velocity) override {
    return checker_->isGoalReached(pose, goal, velocity);
  }
  bool getTolerances(geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & velocity) override {
    if (!checker_->getTolerances(pose, velocity)) {return false;}
    pose.position.x = ReturnPhase::entry_tolerance(pose.position.x);
    pose.position.y = ReturnPhase::entry_tolerance(pose.position.y);
    return true;
  }
private:
  nav2_core::GoalChecker * checker_;
};

}  // namespace qwen_robotnav_progress
