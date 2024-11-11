#include <cmath>
#include <algorithm>

#include "bumperbot_motion/pd_motion_planner.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bumperbot_motion
{
PDMotionPlanner::PDMotionPlanner() : Node("pd_motion_planner_node"),
    kp_(2.0), kd_(0.1), goal_tolerance_(0.1), max_linear_velocity_(0.3),
    max_angular_velocity_(1.0), prev_angular_error_(0.0), prev_linear_error_(0.0)
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    declare_parameter<double>("kp", kp_);
    declare_parameter<double>("kd", kd_);
    declare_parameter<double>("goal_tolerance", goal_tolerance_);
    declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
    declare_parameter<double>("max_angular_velocity", max_angular_velocity_);
    kp_ = get_parameter("kp").as_double();
    kd_ = get_parameter("kd").as_double();
    goal_tolerance_ = get_parameter("goal_tolerance").as_double();
    max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/a_star/path", 10, std::bind(&PDMotionPlanner::pathCallback, this, std::placeholders::_1));
        
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    closest_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pd/next_pose", 10);
}

void PDMotionPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg)
{
    bool is_first_cycle = true;
    rclcpp::Time last_cycle_time;

    while(rclcpp::ok()) {
        // Get the robot's current pose in the path frame
        try {
            map_to_base_ts_ = tf_buffer_->lookupTransform(
                path_msg->header.frame_id, "base_footprint", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Could not transform: %s", ex.what());
            return;
        }

        geometry_msgs::msg::PoseStamped next_pose;
        getNextPose(path_msg, next_pose);

        double dx = path_msg->poses.front().pose.position.x - map_to_base_ts_.transform.translation.x;
        double dy = path_msg->poses.front().pose.position.y - map_to_base_ts_.transform.translation.y;
        double distance_to_goal = std::sqrt(dx * dx + dy * dy);
        if(distance_to_goal <= goal_tolerance_){
            RCLCPP_INFO(get_logger(), "Goal Reached!");
            break;
        }

        // Transform the closest point into the robot's frame
        tf2::Transform map_to_base_tf, map_to_next_pose_tf;
        tf2::fromMsg(map_to_base_ts_.transform, map_to_base_tf);
        tf2::fromMsg(next_pose.pose, map_to_next_pose_tf);

        geometry_msgs::msg::PoseStamped next_pose_robot_frame;
        next_pose_robot_frame.header.frame_id = "base_footprint";
        tf2::toMsg(map_to_base_tf.inverse() * map_to_next_pose_tf, next_pose_robot_frame.pose);
        closest_pub_->publish(next_pose_robot_frame);

        // Calculate the PDMotionPlanner command
        double dt;
        if (is_first_cycle) {
            dt = 0.0;
            is_first_cycle = false;
            last_cycle_time = get_clock()->now();
            continue;
        } else {
            dt = (get_clock()->now() - last_cycle_time).seconds();
        }

        double angular_error = std::atan2(next_pose_robot_frame.pose.position.y,
            next_pose_robot_frame.pose.position.x);
        double angular_error_derivative = (angular_error - prev_angular_error_) / dt;
        double linear_error = next_pose_robot_frame.pose.position.x;
        double linear_error_derivative = (linear_error - prev_linear_error_) / dt;

        double command_angular = std::clamp(kp_ * angular_error + kd_ * angular_error_derivative,
            -max_angular_velocity_, max_angular_velocity_);
        double command_linear = std::clamp(kp_ * linear_error + kd_ * linear_error_derivative,
            -max_linear_velocity_, max_linear_velocity_);

        last_cycle_time = get_clock()->now();
        prev_angular_error_ = angular_error;
        prev_linear_error_ = linear_error;
        
        // Create and publish the velocity command
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = command_linear;
        cmd_vel.angular.z = command_angular;
        cmd_pub_->publish(cmd_vel);
    }
}

void PDMotionPlanner::getNextPose(const nav_msgs::msg::Path::SharedPtr path, geometry_msgs::msg::PoseStamped & next_pose)
{
    // Find the closest point to the robot on the path
    for(const auto & pose : path->poses){
        double dx = pose.pose.position.x - map_to_base_ts_.transform.translation.x;
        double dy = pose.pose.position.y - map_to_base_ts_.transform.translation.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < goal_tolerance_) {
            next_pose = pose;
            break;
        }
    }
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