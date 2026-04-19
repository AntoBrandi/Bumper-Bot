#ifndef BALL_FOLLOWER_HPP
#define BALL_FOLLOWER_HPP

#include <memory>
#include <string>
#include <vector>

#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_filter.hpp"

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::placeholders;

namespace bumperbot_navigation
{
class BallFollower : public rclcpp::Node
{
public:
  BallFollower(const std::string & name);

private:
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;
  std::shared_ptr<ApproximateSync> approximate_sync_;

  image_transport::Publisher detection_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ball_pose_pub_;

  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);

  std::string camera_name_;
  std::vector<int64_t> lower_hsv_;
  std::vector<int64_t> upper_hsv_;
  double ball_radius_;
};
}  // namespace bumperbot_navigation

#endif  // BALL_FOLLOWER_HPP