#ifndef VISUAL_ODOMETRY_HPP
#define VISUAL_ODOMETRY_HPP

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

#include "nav_msgs/msg/odometry.hpp"

using namespace std::placeholders;

namespace bumperbot_vision
{
class FeatureMethod
{
public:
  FeatureMethod();

  void compute(const cv::Mat & img, cv::Mat & img_match);

  cv::Mat getTransform(const cv::Mat & K);

private:
  cv::Mat prev_image_;
  std::vector<cv::KeyPoint> prev_keypoints_, curr_keypoints_;
  cv::Mat prev_descriptor_;
  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> descriptor_;
  cv::Ptr<cv::DescriptorMatcher> matcher_;
  std::vector<cv::DMatch> good_matches_;
};


class DirectMethod
{
public:
  DirectMethod();

  void compute(const cv::Mat & img, cv::Mat & img_match);

  cv::Mat getTransform(const cv::Mat & K);

private:
  cv::Mat prev_image_;
  std::vector<cv::KeyPoint> prev_keypoints_;
  cv::Ptr<cv::GFTTDetector> detector_;
  std::vector<cv::Point2f> pts_prev_, pts_curr_;
  std::vector<uchar> status_;
};

class VisualOdometry : public rclcpp::Node
{
public:
  VisualOdometry(const std::string & name);

private:
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;
  std::shared_ptr<ApproximateSync> approximate_sync_;

  image_transport::Publisher matches_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  bool use_direct_method_;
  std::unique_ptr<FeatureMethod> feature_method_;
  std::unique_ptr<DirectMethod> direct_method_;

  cv::Mat world_pose_;
  rclcpp::Time last_frame_time_;
  bool first_frame_ = true;

  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);

  void calculateOdometry(const cv::Mat & T_rel, const double dt);
};
}  // namespace bumperbot_vision

#endif  // VISUAL_ODOMETRY_HPP