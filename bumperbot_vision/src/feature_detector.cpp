#include "bumperbot_vision/feature_detector.hpp"

#include "cv_bridge/cv_bridge.hpp"

using namespace std::placeholders;

namespace bumperbot_vision
{
FeatureDetector::FeatureDetector(const std::string & name)
: Node(name)
{
  approximate_sync_ = std::make_shared<ApproximateSync>(
    ApproximatePolicy(10), image_sub_, info_sub_);
  approximate_sync_->registerCallback(
    std::bind(&FeatureDetector::imageCallback, this, _1, _2));
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

void FeatureDetector::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  // Camera Intrinsics
  K_ = (cv::Mat_<double>(3, 3) << 
    info_msg->k[0], info_msg->k[1], info_msg->k[2],
    info_msg->k[3], info_msg->k[4], info_msg->k[5],
    info_msg->k[6], info_msg->k[7], info_msg->k[8]);

  cv::Mat img = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
  std::vector<cv::KeyPoint> curr_keypoints;
  detector_->detect(img, curr_keypoints);
  cv::Mat curr_descriptor;
  descriptor_->compute(img, curr_keypoints, curr_descriptor);

  if (prev_image_.empty()) {
    // Initialize if first frame
    prev_image_ = img.clone();
    prev_keypoints_ = curr_keypoints;
    prev_descriptor_ = curr_descriptor.clone();
    return;
  }

  std::vector<cv::DMatch> matches, good_matches;
  matcher_->match(prev_descriptor_, curr_descriptor, matches);

  auto min_max = std::minmax_element(matches.begin(), matches.end(),
    [](const cv::DMatch & m1, const cv::DMatch & m2) { return m1.distance < m2.distance; });
  double min_dist = min_max.first->distance;

  for(const auto& m : matches) {
    if(m.distance <= std::max(2 * min_dist, 30.0)) {
      good_matches.push_back(m);
    }
  }

  cv::Mat img_match;
  cv::drawMatches(prev_image_, prev_keypoints_, img, curr_keypoints, good_matches, img_match);

  // Publish the matches image
  auto msg_matches = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", img_match).toImageMsg();
  msg_matches->header.frame_id = image_msg->header.frame_id;
  matches_pub_.publish(*msg_matches);

  cv::Mat R, t;
  estimatePose(curr_keypoints, good_matches, R, t);
  RCLCPP_INFO_STREAM(get_logger(), "Estimated Rotation:\n" << R << "\nEstimated Translation:\n" << t);

  prev_image_ = img.clone();
  prev_keypoints_ = curr_keypoints;
  prev_descriptor_ = curr_descriptor.clone();
}

void FeatureDetector::estimatePose(
  const std::vector<cv::KeyPoint> & curr_keypoints,
  const std::vector<cv::DMatch> & matches,
  cv::Mat & R, cv::Mat & t)
{
  std::vector<cv::Point2f> points_1;
  std::vector<cv::Point2f> points_2;

  for (size_t i = 0; i < matches.size(); i++){
    points_1.push_back(prev_keypoints_.at(matches.at(i).queryIdx).pt);
    points_2.push_back(curr_keypoints.at(matches.at(i).trainIdx).pt);
  }

  // Calculate the Essential Matrix
  cv::Mat essential_matrix = cv::findEssentialMat(points_1, points_2, K_);

  // Calculate the Homography Matrix
  cv::Mat homography_matrix;
  homography_matrix = cv::findHomography(points_1, points_2, cv::RANSAC, 3);
  cv::recoverPose(essential_matrix, points_1, points_2, K_, R, t);
}
}  // namespace bumperbot_vision

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bumperbot_vision::FeatureDetector>("feature_detector"));
  rclcpp::shutdown();
  return 0;
}