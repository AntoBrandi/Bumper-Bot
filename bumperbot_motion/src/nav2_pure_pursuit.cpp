#include <algorithm>

#include "nav2_util/node_utils.hpp"
#include "bumperbot_motion/nav2_pure_pursuit.hpp"

namespace bumperbot_motion
{
void PurePursuit::configure(
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
    node, plugin_name_ + ".look_ahead_distance",
    rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_velocity",
    rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_velocity",
    rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_tolerance",
    rclcpp::ParameterValue(0.1));

  node->get_parameter(plugin_name_ + ".look_ahead_distance", look_ahead_distance_);
  node->get_parameter(plugin_name_ + ".max_linear_velocity", max_linear_velocity_);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);
  node->get_parameter(plugin_name_ + ".goal_tolerance", goal_tolerance_);

  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("pure_pursuit/carrot", 1);
}

void PurePursuit::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up PurePursuit");
  carrot_pub_.reset();
}

void PurePursuit::activate()
{
  RCLCPP_INFO(logger_, "Activating PurePursuit");
  carrot_pub_->on_activate();
}

void PurePursuit::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating PurePursuit");
  carrot_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped PurePursuit::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  RCLCPP_INFO_STREAM(logger_, "Robot Pose frame:" << robot_pose.header.frame_id);
  RCLCPP_INFO_STREAM(logger_, "Robot Pose x:" << robot_pose.pose.position.x);
  RCLCPP_INFO_STREAM(logger_, "Robot Pose y:" << robot_pose.pose.position.y);
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  if(global_plan_.poses.empty()){
    return cmd_vel;
  }

  geometry_msgs::msg::PoseStamped carrot_pose = getCarrotPose(robot_pose);

  double dx = carrot_pose.pose.position.x - robot_pose.pose.position.x;
  double dy = carrot_pose.pose.position.y - robot_pose.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  if(distance <= goal_tolerance_){
    RCLCPP_INFO(logger_, "Goal Reached!");
    return cmd_vel;
  }

  // Transform the look-ahead point into the robot's frame
  tf2::Transform map_to_base_tf, map_to_carrot_pose_tf;
  tf2::fromMsg(robot_pose.pose, map_to_base_tf);
  tf2::fromMsg(carrot_pose.pose, map_to_carrot_pose_tf);

  geometry_msgs::msg::PoseStamped carrot_pose_robot_frame;
  carrot_pose_robot_frame.header.frame_id = "base_footprint";
  tf2::toMsg(map_to_base_tf.inverse() * map_to_carrot_pose_tf, carrot_pose_robot_frame.pose);
  carrot_pub_->publish(carrot_pose_robot_frame);
        
  // Calculate the curvature to the look-ahead point
  double curvature = getCurvature(carrot_pose_robot_frame.pose);
        
  // Create and publish the velocity command
  cmd_vel.twist.linear.x = max_linear_velocity_;
  cmd_vel.twist.angular.z = curvature * max_angular_velocity_;

  return cmd_vel;
}

void PurePursuit::setPlan(const nav_msgs::msg::Path & path)
{
  RCLCPP_INFO_STREAM(logger_, "Path received with " << path.poses.size() << " poses");
  RCLCPP_INFO_STREAM(logger_, "Path frame " << path.header.frame_id);
  global_plan_ = path;
}

void PurePursuit::setSpeedLimit(const double &, const bool &){}

geometry_msgs::msg::PoseStamped PurePursuit::getCarrotPose(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Find the look-ahead point on the path
  for(const auto & pose : global_plan_.poses){
    double dx = pose.pose.position.x - robot_pose.pose.position.x;
    double dy = pose.pose.position.y - robot_pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    if(distance < look_ahead_distance_){

      return pose;
    }
  }
  return geometry_msgs::msg::PoseStamped();
}

double PurePursuit::getCurvature(const geometry_msgs::msg::Pose & carrot_pose)
{
  const double carrot_dist2 =
  (carrot_pose.position.x * carrot_pose.position.x) +
  (carrot_pose.position.y * carrot_pose.position.y);
    
  // Find curvature of circle (k = 1 / R)
  if (carrot_dist2 > 0.001) {
    return 2.0 * carrot_pose.position.y / carrot_dist2;
  } else {
    return 0.0;
  }
}

}  // namespace bumperbot_motion

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bumperbot_motion::PurePursuit, nav2_core::Controller)