#include "bumperbot_vision/undistort_image.hpp"

#include "cv_bridge/cv_bridge.hpp"

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

namespace bumperbot_vision
{
UndistortImage::UndistortImage(const std::string & name) : Node(name)
{
  approximate_sync_ = std::make_shared<ApproximateSync>(
    ApproximatePolicy(10), image_sub_, info_sub_);
  approximate_sync_->registerCallback(
    std::bind(&UndistortImage::imageCallback, this, _1, _2));
  image_transport::TransportHints hints{this};
  const auto sensor_data_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();

  // Declare parameters
  camera_name_ = declare_parameter("camera_name", "left_camera");

  image_sub_.subscribe(
    this, camera_name_ + "/image_raw", hints.getTransport(), sensor_data_qos);
  info_sub_.subscribe(this, camera_name_ + "/camera_info", sensor_data_qos);

  undistort_image_pub_ =
    image_transport::create_publisher(this, camera_name_ + "/image_rect", rmw_qos_profile_sensor_data);
}

void UndistortImage::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  cv::Mat image = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
  int rows = image.rows, cols = image.cols;
  cv::Mat image_undistort;

  model_.fromCameraInfo(info_msg);
  model_.rectifyImage(image, image_undistort, 1);

  auto undistort_msg =
    cv_bridge::CvImage(image_msg->header, image_msg->encoding, image_undistort).toImageMsg();
  undistort_msg->header = image_msg->header;
  undistort_image_pub_.publish(undistort_msg);
}
}  // namespace bumperbot_vision

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bumperbot_vision::UndistortImage>("undistort_image"));
  rclcpp::shutdown();
  return 0;
}