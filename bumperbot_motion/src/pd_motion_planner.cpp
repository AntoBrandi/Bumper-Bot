#include <chrono>
#include <algorithm>

#include "bumperbot_motion/pd_motion_planner.hpp"

#include "tf2/utils.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bumperbot_motion
{
PDMotionPlanner::PDMotionPlanner() : Node("pd_motion_planner_node"),
  kp_(2.0), kd_(0.1), step_size_(0.2), max_linear_velocity_(0.3),
  max_angular_velocity_(1.0), prev_angular_error_(0.0), prev_linear_error_(0.0)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  declare_parameter<double>("kp", kp_);
  declare_parameter<double>("kd", kd_);
  declare_parameter<double>("step_size", step_size_);
  declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
  declare_parameter<double>("max_angular_velocity", max_angular_velocity_);
  kp_ = get_parameter("kp").as_double();
  kd_ = get_parameter("kd").as_double();
  step_size_ = get_parameter("step_size").as_double();
  max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/a_star/path", 10, std::bind(&PDMotionPlanner::pathCallback, this, std::placeholders::_1));
        
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  next_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pd/next_pose", 10);
  control_loop_ = create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&PDMotionPlanner::controlLoop, this));
  last_cycle_time_ = get_clock()->now();
}

void PDMotionPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
  global_plan_ = *path;
}

void PDMotionPlanner::controlLoop()
{
  if(global_plan_.poses.empty()){
    return;
  }

  // Get the robot's current pose in the odom frame
  geometry_msgs::msg::TransformStamped robot_pose;
  try {
    robot_pose = tf_buffer_->lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Could not transform: %s", ex.what());
    return;
  }

  if(!transformPlan(robot_pose.header.frame_id)){
    RCLCPP_ERROR(get_logger(), "Unable to transform Plan in robot's frame");
    return;
  }

  geometry_msgs::msg::PoseStamped robot_pose_stamped;
  robot_pose_stamped.header.frame_id = robot_pose.header.frame_id;
  robot_pose_stamped.pose.position.x = robot_pose.transform.translation.x;
  robot_pose_stamped.pose.position.y = robot_pose.transform.translation.y;
  robot_pose_stamped.pose.orientation = robot_pose.transform.rotation;
  auto next_pose = getNextPose(robot_pose_stamped);

  double dx = next_pose.pose.position.x - robot_pose_stamped.pose.position.x;
  double dy = next_pose.pose.position.y - robot_pose_stamped.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  if(distance <= 0.1){
    RCLCPP_INFO(get_logger(), "Goal Reached!");
    global_plan_.poses.clear();
    return;
  }
  
  next_pose_pub_->publish(next_pose);
        
  // Calculate the PDMotionPlanner command
  tf2::Transform next_pose_robot_tf, robot_tf, next_pose_tf;
  tf2::fromMsg(robot_pose_stamped.pose, robot_tf);
  tf2::fromMsg(next_pose.pose, next_pose_tf);
  next_pose_robot_tf = robot_tf.inverse() * next_pose_tf;

  double dt = (get_clock()->now() - last_cycle_time_).seconds();

  double angular_error = next_pose_robot_tf.getOrigin().getY();
  double angular_error_derivative = (angular_error - prev_angular_error_) / dt;
  double linear_error = next_pose_robot_tf.getOrigin().getX();
  double linear_error_derivative = (linear_error - prev_linear_error_) / dt;

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.angular.z = std::clamp(kp_ * angular_error + kd_ * angular_error_derivative,
    -max_angular_velocity_, max_angular_velocity_);
  cmd_vel.linear.x = std::clamp(kp_ * linear_error + kd_ * linear_error_derivative,
    -max_linear_velocity_, max_linear_velocity_);
  
  last_cycle_time_ = get_clock()->now();
  prev_angular_error_ = angular_error;
  prev_linear_error_ = linear_error;

  cmd_pub_->publish(cmd_vel);
}

geometry_msgs::msg::PoseStamped PDMotionPlanner::getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  geometry_msgs::msg::PoseStamped next_pose = global_plan_.poses.back();
  for (auto pose_it = global_plan_.poses.rbegin(); pose_it != global_plan_.poses.rend(); ++pose_it) {
    double dx = pose_it->pose.position.x - robot_pose.pose.position.x;
    double dy = pose_it->pose.position.y - robot_pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    if(distance > step_size_){
      next_pose = *pose_it;
    } else {
      break;
    }
  }
  return next_pose;
}

bool PDMotionPlanner::transformPlan(const std::string & frame)
{
  if(global_plan_.header.frame_id == frame){
    return true;
  }
  geometry_msgs::msg::TransformStamped transform;
  try{
    transform = tf_buffer_->lookupTransform(frame, global_plan_.header.frame_id, tf2::TimePointZero);
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "Couldn't transform plan from frame " <<
      global_plan_.header.frame_id << " to frame " << frame);
    return false;
  }
  for(auto & pose : global_plan_.poses){
    tf2::doTransform(pose, pose, transform);
  }
  global_plan_.header.frame_id = frame;
  return true;
}
}  // namespace bumperbot_motion

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bumperbot_motion::PDMotionPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}