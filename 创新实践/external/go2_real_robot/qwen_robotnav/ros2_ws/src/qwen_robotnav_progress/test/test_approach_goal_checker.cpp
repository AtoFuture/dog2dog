#include <cmath>
#include <stdexcept>
#include "qwen_robotnav_progress/approach_goal_checker.hpp"

struct ActualChecker : nav2_core::GoalChecker {
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Pose & goal,
    const geometry_msgs::msg::Twist &) override {
    return std::hypot(pose.position.x-goal.position.x,pose.position.y-goal.position.y)<=.25;
  }
  bool getTolerances(geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist &) override {
    pose.position.x=.25;pose.position.y=.25;return true;
  }
};

int main() {
  ActualChecker actual;
  qwen_robotnav_progress::ApproachGoalChecker approach(&actual);
  geometry_msgs::msg::Pose tolerance, pose, goal;
  geometry_msgs::msg::Twist velocity;
  if (!approach.getTolerances(tolerance,velocity) || tolerance.position.x!=.05 ||
    tolerance.position.y!=.05) {throw std::runtime_error("RPP final rotation started too early");}
  actual.getTolerances(tolerance,velocity);
  if (tolerance.position.x!=.25) {throw std::runtime_error("Success tolerance was changed");}
  pose.position.x=.20;
  if (!approach.isGoalReached(pose,goal,velocity)) {throw std::runtime_error("Success checker was narrowed");}
  pose.position.x=.30;
  if (approach.isGoalReached(pose,goal,velocity)) {throw std::runtime_error("Success tolerance was weakened");}
}
