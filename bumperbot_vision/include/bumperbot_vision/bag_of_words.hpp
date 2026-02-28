#ifndef BAG_OF_WORDS_HPP
#define BAG_OF_WORDS_HPP

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

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "bumperbot_msgs/srv/set_string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

#include "DBoW3/DBoW3.h"

namespace bumperbot_vision
{
class BagOfWords : public rclcpp::Node
{
public:
  BagOfWords(const std::string & name);

private:
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;
  std::shared_ptr<ApproximateSync> approximate_sync_;

  bool is_training_;
  bool is_trained_;
  double training_frequency_;
  rclcpp::Time last_train_time_;

  cv::Ptr<cv::Feature2D> detector_;
  std::vector<cv::Mat> training_features_;
  std::vector<geometry_msgs::msg::TransformStamped> stored_poses_;
  
  DBoW3::Vocabulary voc_;
  DBoW3::Database db_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr train_mode_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_mode_srv_;
  rclcpp::Service<bumperbot_msgs::srv::SetString>::SharedPtr save_srv_;
  rclcpp::Service<bumperbot_msgs::srv::SetString>::SharedPtr load_srv_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);

  bool buildVocabulary();
};
}  // namespace bumperbot_vision

#endif  // BAG_OF_WORDS_HPP