#ifndef UNDISTORT_IMAGE_HPP
#define UNDISTORT_IMAGE_HPP

#include <memory>
#include <string>

#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_filter.hpp"

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "image_geometry/pinhole_camera_model.hpp"

using namespace std::placeholders;

namespace bumperbot_vision
{
class UndistortImage : public rclcpp::Node
{
public:
  UndistortImage(const std::string & name);

private:
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;
  std::shared_ptr<ApproximateSync> approximate_sync_;

  image_transport::Publisher undistort_image_pub_;

  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);

  image_geometry::PinholeCameraModel model_;
  std::string camera_name_;
};
}  // namespace bumperbot_vision

#endif  // UNDISTORT_IMAGE_HPP