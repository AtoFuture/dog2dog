#include <algorithm>
#include <cmath>
#include <chrono>
#include <mutex>
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "qwen_robotnav_progress/return_phase.hpp"
#include "qwen_robotnav_progress/approach_goal_checker.hpp"

namespace qwen_robotnav_progress {
class ReturnController : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController {
  using Base = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    Base::configure(parent, name, tf, costmap_ros);
    auto node = parent.lock();
    if (!node) {throw nav2_core::PlannerException("Missing RETURN lifecycle node");}
    const auto parameter = name + ".terminal_turn_handoff_enabled";
    if (!node->has_parameter(parameter)) {
      node->declare_parameter(parameter, rclcpp::ParameterValue(false));
    }
    node->get_parameter(parameter, terminal_turn_handoff_enabled_);
  }
  void activate() override {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    resetPhase(); has_goal_ = false; Base::activate();
  }
  void deactivate() override {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    resetPhase(); has_goal_ = false; Base::deactivate();
  }
  void setPlan(const nav_msgs::msg::Path & path) override {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    if (path.poses.empty()) {throw nav2_core::PlannerException("Empty RETURN path");}
    auto endpoint = path.poses.back();
    if (endpoint.header.frame_id.empty()) {endpoint.header.frame_id = path.header.frame_id;}
    if (!has_goal_ || endpoint.header.frame_id != goal_.header.frame_id ||
      std::hypot(endpoint.pose.position.x - goal_.pose.position.x,
      endpoint.pose.position.y - goal_.pose.position.y) > 1.e-4 ||
      std::abs(std::remainder(tf2::getYaw(endpoint.pose.orientation) -
      tf2::getYaw(goal_.pose.orientation), 2. * M_PI)) > 1.e-4) {
      resetPhase();
    }
    goal_ = endpoint; has_goal_ = true;
    Base::setPlan(path);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * checker) override
  {
    std::lock_guard<std::mutex> phase_lock(phase_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (now - last_compute_ > std::chrono::seconds(1)) {resetPhase();}
    last_compute_ = now;
    if (!has_goal_ || !checker) {throw nav2_core::PlannerException("Missing RETURN goal/checker");}
    geometry_msgs::msg::Pose tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    if (!checker->getTolerances(tolerance, velocity_tolerance)) {
      throw nav2_core::PlannerException("Missing RETURN tolerances");
    }
    // The endpoint, not the constantly changing lookahead carrot, determines
    // entry to final alignment. Use the current pose time for the transform.
    auto target = goal_;
    target.header.stamp = pose.header.stamp;
    geometry_msgs::msg::PoseStamped local_goal;
    if (!transformPose(pose.header.frame_id, target, local_goal)) {
      throw nav2_core::PlannerException("Cannot transform RETURN endpoint");
    }
    const double distance = std::hypot(local_goal.pose.position.x - pose.pose.position.x,
      local_goal.pose.position.y - pose.pose.position.y);
    const double yaw_error = std::remainder(tf2::getYaw(local_goal.pose.orientation) -
      tf2::getYaw(pose.pose.orientation), 2. * M_PI);
    const double endpoint_bearing = std::remainder(std::atan2(
      local_goal.pose.position.y - pose.pose.position.y,
      local_goal.pose.position.x - pose.pose.position.x) - tf2::getYaw(pose.pose.orientation),
      2. * M_PI);
    bool approach_requires_turn;
    {
      std::lock_guard<std::mutex> params_lock(mutex_);
      approach_requires_turn = std::abs(endpoint_bearing) > rotate_to_heading_min_angle_;
    }
    ReturnPhase::Stage stage;
    try {
      stage = phase_.update(distance, yaw_error, tolerance.position.x,
        std::abs(tf2::getYaw(tolerance.orientation)),
        terminal_turn_handoff_enabled_ && approach_requires_turn);
    } catch (const std::invalid_argument & error) {
      throw nav2_core::PlannerException(error.what());
    }
    if (!has_logged_stage_ || stage != logged_stage_) {
      if (stage == ReturnPhase::Stage::Align) {
        settle_until_ = now + std::chrono::milliseconds(600);
      }
      RCLCPP_INFO(logger_, "RETURN phase=%s distance=%.3f yaw_error=%.3f path_turn=%s",
        stage == ReturnPhase::Stage::Approach ? "approach" :
        stage == ReturnPhase::Stage::Align ? "align" : "hold", distance, yaw_error,
        approach_requires_turn ? "true" : "false");
      logged_stage_ = stage;
      has_logged_stage_ = true;
    }
    if (stage == ReturnPhase::Stage::Approach) {
      ApproachGoalChecker approach_checker(checker);
      return Base::computeVelocityCommands(pose, velocity, &approach_checker);
    }
    // Keep the original RPP rotation acceleration and projected footprint
    // collision checks. The downstream ramp and final guard remain in force.
    std::lock_guard<std::mutex> params_lock(mutex_);
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap_->getMutex());
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    if (stage == ReturnPhase::Stage::Align && now >= settle_until_) {
      rotateToHeading(command.twist.linear.x, command.twist.angular.z, yaw_error, velocity);
    }
    if (isCollisionImminent(pose, command.twist.linear.x, command.twist.angular.z,
      std::max(distance, lookahead_dist_))) {
      throw nav2_core::PlannerException("Collision during RETURN alignment");
    }
    return command;
  }
private:
  void resetPhase() {phase_.reset(); has_logged_stage_ = false;}
  std::mutex phase_mutex_;
  ReturnPhase phase_;
  geometry_msgs::msg::PoseStamped goal_;
  bool has_goal_{false};
  bool terminal_turn_handoff_enabled_{false};
  ReturnPhase::Stage logged_stage_{ReturnPhase::Stage::Approach};
  bool has_logged_stage_{false};
  std::chrono::steady_clock::time_point last_compute_{};
  std::chrono::steady_clock::time_point settle_until_{};
};
}  // namespace qwen_robotnav_progress
PLUGINLIB_EXPORT_CLASS(qwen_robotnav_progress::ReturnController, nav2_core::Controller)
