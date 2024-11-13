#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace bumperbot_motion
{
class PurePursuit : public rclcpp::Node
{
public:
    PurePursuit();

private:
    rclcpp::TimerBase::SharedPtr control_loop_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pub_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double look_ahead_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;

    nav_msgs::msg::Path global_plan_;

    void controlLoop();

    bool transformPlan(const std::string & frame);

    void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

    geometry_msgs::msg::PoseStamped getCarrotPose(
        const geometry_msgs::msg::PoseStamped & robot_pose);

    double getCurvature(const geometry_msgs::msg::Pose & carrot_pose);
};
}  // namespace bumperbot_motion

#endif // PURE_PURSUIT_HPP