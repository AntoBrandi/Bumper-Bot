#include <algorithm>

#include "nav2_util/node_utils.hpp"
#include "bumperbot_motion/nav2_pd_motion_planner.hpp"

namespace bumperbot_motion
{
void PDMotionPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;

  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf_buffer;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp",
    rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kd",
    rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_velocity",
    rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_velocity",
    rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_tolerance",
    rclcpp::ParameterValue(0.1));

  node->get_parameter(plugin_name_ + ".kp", kp_);
  node->get_parameter(plugin_name_ + ".kd", kd_);
  node->get_parameter(plugin_name_ + ".max_linear_velocity", max_linear_velocity_);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);
  node->get_parameter(plugin_name_ + ".goal_tolerance", goal_tolerance_);

  next_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("pd/next_pose", 1);
}

void PDMotionPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up PDMotionPlanner");
  next_pose_pub_.reset();
}

void PDMotionPlanner::activate()
{
  auto node = node_.lock();
  RCLCPP_INFO(logger_, "Activating PDMotionPlanner");
  next_pose_pub_->on_activate();
  last_cycle_time_ = node->get_clock()->now();
}

void PDMotionPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating PDMotionPlanner");
  next_pose_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped PDMotionPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  auto node = node_.lock();
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;

  if(global_plan_.poses.empty()){
    return cmd_vel;
  }

  geometry_msgs::msg::PoseStamped robot_pose_transfomed = transformPose(robot_pose, global_plan_.header.frame_id);
  geometry_msgs::msg::PoseStamped next_pose = getNextPose(robot_pose_transfomed);

  // double dx = global_plan_.poses.front().pose.position.x - robot_pose.pose.position.x;
  // double dy = global_plan_.poses.front().pose.position.y - robot_pose.pose.position.y;
  // double distance_to_goal = std::sqrt(dx * dx + dy * dy);
  // if(distance_to_goal <= goal_tolerance_){
  //   RCLCPP_INFO(logger_, "Goal Reached!");
  //   return cmd_vel;
  // }

  next_pose_pub_->publish(next_pose);
        
  // Calculate the PDMotionPlanner command
  double dt = (node->get_clock()->now() - last_cycle_time_).seconds();

  double angular_error = std::atan2(next_pose.pose.position.y,
    next_pose.pose.position.x);
  double angular_error_derivative = (angular_error - prev_angular_error_) / dt;
  double linear_error = next_pose.pose.position.x - robot_pose_transfomed.pose.position.x;
  double linear_error_derivative = (linear_error - prev_linear_error_) / dt;

  double command_angular = std::clamp(kp_ * angular_error + kd_ * angular_error_derivative,
      -max_angular_velocity_, max_angular_velocity_);
  double command_linear = std::clamp(kp_ * linear_error + kd_ * linear_error_derivative,
      -max_linear_velocity_, max_linear_velocity_);
  last_cycle_time_ = node->get_clock()->now();
  prev_angular_error_ = angular_error;
  prev_linear_error_ = linear_error;
        
  // Create and publish the velocity command
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = command_linear;
  // cmd_vel.twist.angular.z = command_angular;

  return cmd_vel;
}

void PDMotionPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void PDMotionPlanner::setSpeedLimit(const double &, const bool &){}

geometry_msgs::msg::PoseStamped PDMotionPlanner::getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Find the look-ahead point on the path
  geometry_msgs::msg::PoseStamped next_pose;
  for(auto it = global_plan_.poses.rbegin(); it != global_plan_.poses.rend(); ++it){
    double dx = it->pose.position.x - robot_pose.pose.position.x;
    double dy = it->pose.position.y - robot_pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    if(distance < goal_tolerance_){
      next_pose = *it;
      break;
    }
  }
  return next_pose;
}

geometry_msgs::msg::PoseStamped PDMotionPlanner::transformPose(const geometry_msgs::msg::PoseStamped & pose,
  const std::string & frame)
{
  if(frame == pose.header.frame_id){
    return pose;
  }

  geometry_msgs::msg::PoseStamped transformed_pose = pose;
  geometry_msgs::msg::TransformStamped robot_to_path_ts;
  try {
    robot_to_path_ts = tf_buffer_->lookupTransform(
      frame, pose.header.frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(logger_, "Could not transform: %s", ex.what());
    return transformed_pose;
  }
  tf2::Transform pose_tf, robot_to_path_tf;
  tf2::fromMsg(pose.pose, pose_tf);
  tf2::fromMsg(robot_to_path_ts.transform, robot_to_path_tf);
  tf2::toMsg(pose_tf * robot_to_path_tf, transformed_pose.pose);
  transformed_pose.header.frame_id = frame;
  return transformed_pose;
}

}  // namespace bumperbot_motion

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bumperbot_motion::PDMotionPlanner, nav2_core::Controller)