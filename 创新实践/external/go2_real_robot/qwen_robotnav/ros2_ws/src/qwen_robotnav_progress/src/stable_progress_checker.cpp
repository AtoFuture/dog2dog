#include <memory>
#include <string>
#include "nav2_core/progress_checker.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "qwen_robotnav_progress/stable_progress.hpp"

namespace qwen_robotnav_progress
{
class StableProgressChecker : public nav2_core::ProgressChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name) override
  {
    auto node = parent.lock();
    if (!node) {throw std::runtime_error("Progress checker parent expired");}
    clock_ = node->get_clock(); logger_ = node->get_logger();
    buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, node, true);
    const auto parameter = [&](const std::string & suffix, double default_value) {
        const auto key = name + "." + suffix;
        if (!node->has_parameter(key)) {node->declare_parameter(key, default_value);}
        return node->get_parameter(key).as_double();
      };
    progress_ = std::make_unique<StableProgress>(
      parameter("required_movement_radius", .25),
      parameter("movement_time_allowance", 12.), parameter("median_window_s", 2.));
    RCLCPP_INFO(logger_, "Stable map progress checker enabled; leg odometry speed is not progress");
  }

  bool check(geometry_msgs::msg::PoseStamped & pose) override
  {
    // ControllerServer passes a LOCAL costmap pose (odom in this project).
    // Transform it to map; checking odom would still count slipping legs as motion.
    geometry_msgs::msg::PoseStamped map_pose;
    try {
      if (pose.header.frame_id == "map") {map_pose = pose;}
      else {
        const auto transform = buffer_->lookupTransform("map", pose.header.frame_id,
            tf2::TimePointZero);
        const double age = (clock_->now() - rclcpp::Time(transform.header.stamp)).seconds();
        if (age > 1. || age < -1.) {throw std::runtime_error("Stale map transform");}
        tf2::doTransform(pose, map_pose, transform);
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Map progress transform unavailable: %s", error.what());
      return false;
    }
    if (!source_logged_) {
      RCLCPP_INFO(logger_, "Progress pose frame: %s; stable evaluation frame: map", pose.header.frame_id.c_str());
      source_logged_ = true;
    }
    const bool ok = progress_->check(clock_->now().seconds(), map_pose.pose.position.x,
        map_pose.pose.position.y);
    if (!ok && !stalled_) {
      RCLCPP_WARN(logger_, "Stable map progress stalled; requesting FollowPath recovery");
    }
    stalled_ = !ok;
    return ok;
  }

  void reset() override {if (progress_) {progress_->reset();} stalled_ = false;}

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("stable_progress")};
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<StableProgress> progress_;
  bool stalled_{false};
  bool source_logged_{false};
};
}  // namespace qwen_robotnav_progress
PLUGINLIB_EXPORT_CLASS(qwen_robotnav_progress::StableProgressChecker, nav2_core::ProgressChecker)
