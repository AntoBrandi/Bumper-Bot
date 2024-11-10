#ifndef PD_HPP
#define PD_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace bumperbot_planning
{
class PD : public rclcpp::Node
{
public:
    PD();

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr closest_pub_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped map_to_base_ts_;

    double kp_;
    double kd_;
    double goal_tolerance_;
    double max_linear_velocity_;
    double max_angular_velocity_;

    double prev_angular_error_;
    double prev_linear_error_;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg);

    void getNextPose(const nav_msgs::msg::Path::SharedPtr path,
        geometry_msgs::msg::PoseStamped & next_pose);
};
}  // namespace bumperbot_planning

#endif // PD_HPP