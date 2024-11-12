#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace bumperbot_motion
{
class PurePursuit : public nav2_core::Controller
{
public:
  PurePursuit() = default;
  ~PurePursuit() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  geometry_msgs::msg::PoseStamped getCarrotPose(const geometry_msgs::msg::PoseStamped & robot_pose);

  bool transformPlan(const std::string & frame);

  double getCurvature(const geometry_msgs::msg::Pose & carrot_pose);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> carrot_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ {rclcpp::get_logger("PurePursuit")};
  rclcpp::Clock::SharedPtr clock_;

  double look_ahead_distance_;
  double max_linear_velocity_;
  double max_angular_velocity_;

  nav_msgs::msg::Path global_plan_;
};

}  // namespace bumperbot_motion

#endif  // PURE_PURSUIT_HPP