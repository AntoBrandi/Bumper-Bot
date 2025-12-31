#include "bumperbot_vision/visual_odometry.hpp"

#include "cv_bridge/cv_bridge.hpp"

namespace bumperbot_vision
{
VisualOdometry::VisualOdometry(const std::string & name) : Node(name)
{
  approximate_sync_ = std::make_shared<ApproximateSync>(
    ApproximatePolicy(10), image_sub_, info_sub_);
  approximate_sync_->registerCallback(
    std::bind(&VisualOdometry::imageCallback, this, _1, _2));
  image_transport::TransportHints hints{this};
  const auto sensor_data_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();

  image_sub_.subscribe(
    this, "left_camera/image_raw", hints.getTransport(), sensor_data_qos);
  info_sub_.subscribe(this, "left_camera/camera_info", sensor_data_qos);

  matches_pub_ =
    image_transport::create_publisher(this, "matches", rmw_qos_profile_sensor_data);

  detector_ = cv::ORB::create();
  descriptor_ = cv::ORB::create();
  matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

void VisualOdometry::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  if (prev_image_.empty()) {
    prev_image_ = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
    detector_->detect(prev_image_, prev_keypoints_);
    descriptor_->compute(prev_image_, prev_keypoints_, prev_descriptor_);
    return;
  }

  cv::Mat image = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;

  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;

  // Detect Oriented FAST keypoints
  detector_->detect(image, keypoints);

  // Compute BRIEF descriptors
  descriptor_->compute(image, keypoints, descriptors);

  // Feature matching using Hamming distance
  std::vector<cv::DMatch> matches;
  matcher_->match(prev_descriptor_, descriptors, matches);

  // sort and remove outliers
  auto min_max = std::minmax_element(
    matches.begin(), matches.end(),
    [](const cv::DMatch & m1, const cv::DMatch & m2) {
      return m1.distance < m2.distance;
    });
  double min_dist = min_max.first->distance;
  double max_dist = min_max.second->distance;

  std::vector<cv::DMatch> good_matches;
  for(size_t i = 0; i < matches.size(); i++) {
    if(matches[i].distance <= std::max(2 * min_dist, 30.0)) {
      good_matches.push_back(matches[i]);
    }
  }

  // Draw the results
  cv::Mat img_match;
  cv::drawMatches(
    prev_image_, prev_keypoints_, image, keypoints, good_matches, img_match);

  // Publish the matches image
  auto msg_matches = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_match).toImageMsg();
  msg_matches->header.frame_id = image_msg->header.frame_id;
  matches_pub_.publish(*msg_matches);

  // Update previous frame data
  prev_image_ = image;
  prev_keypoints_ = keypoints;
  prev_descriptor_ = descriptors;
}
}  // namespace bumperbot_vision

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bumperbot_vision::VisualOdometry>("visual_odometry"));
  rclcpp::shutdown();
  return 0;
}