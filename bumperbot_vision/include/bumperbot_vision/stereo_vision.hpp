#ifndef STEREO_VISION_HPP
#define STEREO_VISION_HPP

#include <memory>
#include <string>

#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_filter.hpp"
#include "point_cloud_transport/point_cloud_transport.hpp"

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::placeholders;

namespace bumperbot_vision
{
class StereoVision : public rclcpp::Node
{
public:
  StereoVision(const std::string & name);

private:
  image_transport::SubscriberFilter left_image_sub_, right_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> left_info_sub_, right_info_sub_;
  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;
  std::shared_ptr<ApproximateSync> approximate_sync_;

  image_transport::Publisher disparity_image_pub_;
  std::unique_ptr<point_cloud_transport::PointCloudTransport> pct_;
  point_cloud_transport::Publisher point_cloud_pub_;

  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & l_image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & l_info_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & r_image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & r_info_msg);

  rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter> & parameters);
  OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  // Stereo Matcher Parameters
  struct SGBMParams {
    int min_disparity;
    int num_disparities;
    int block_size;
    int p1;
    int p2;
    int disp12_max_diff;
    int pre_filter_cap;
    int uniqueness_ratio;
    int speckle_window_size;
    int speckle_range;
    int mode;
  } sgbm_params_;

  double baseline_;
};
}  // namespace bumperbot_vision

#endif  // STEREO_VISION_HPP