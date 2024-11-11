#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace bumperbot_motion
{
class PurePursuit : public rclcpp::Node
{
public:
    PurePursuit();

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pub_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped map_to_base_ts_;

    double look_ahead_distance_;
    double goal_tolerance_;
    double max_linear_velocity_;
    double max_angular_velocity_;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg);

    void getCarrotPose(const nav_msgs::msg::Path::SharedPtr path,
        geometry_msgs::msg::PoseStamped & carrot_pose);

    double getCurvature(const geometry_msgs::msg::Pose & carrot_pose);
};
}  // namespace bumperbot_motion

#endif // PURE_PURSUIT_HPP