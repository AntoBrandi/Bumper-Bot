#ifndef OPTICAL_FLOW_HPP
#define OPTICAL_FLOW_HPP

#include <memory>
#include <string>

#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_filter.hpp"

#include "rclcpp/rclcpp.hpp"

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace bumperbot_vision
{
class OpticalFlow : public rclcpp::Node
{
public:
  OpticalFlow(const std::string & name);

private:
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;
  std::shared_ptr<ApproximateSync> approximate_sync_;

  image_transport::Publisher matches_pub_;

  cv::Mat K_;
  cv::Mat prev_image_;
  std::vector<cv::KeyPoint> prev_keypoints_;
  cv::Ptr<cv::GFTTDetector> detector_;

  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);

  void estimatePose(
    const std::vector<cv::Point2f> & curr_points,
    const std::vector<cv::Point2f> & prev_points,
    cv::Mat & R, cv::Mat & t);
};
}  // namespace bumperbot_vision

#endif  // OPTICAL_FLOW_HPP