#include "bumperbot_vision/optical_flow.hpp"

#include "cv_bridge/cv_bridge.hpp"

using namespace std::placeholders;

namespace bumperbot_vision
{
OpticalFlow::OpticalFlow(const std::string & name)
: Node(name)
{
  approximate_sync_ = std::make_shared<ApproximateSync>(
    ApproximatePolicy(10), image_sub_, info_sub_);
  approximate_sync_->registerCallback(
    std::bind(&OpticalFlow::imageCallback, this, _1, _2));
  image_transport::TransportHints hints{this};
  const auto sensor_data_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();

  image_sub_.subscribe(
    this, "left_camera/image_raw", hints.getTransport(), sensor_data_qos);
  info_sub_.subscribe(this, "left_camera/camera_info", sensor_data_qos);

  matches_pub_ =
    image_transport::create_publisher(this, "matches", rmw_qos_profile_sensor_data);

  detector_ = cv::GFTTDetector::create(500, 0.01, 20);
}

void OpticalFlow::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  // Camera Intrinsics
  K_ = (cv::Mat_<double>(3, 3) << 
    info_msg->k[0], info_msg->k[1], info_msg->k[2],
    info_msg->k[3], info_msg->k[4], info_msg->k[5],
    info_msg->k[6], info_msg->k[7], info_msg->k[8]);

  cv::Mat img;
  try {
    img = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Convert to grayscale for better Optical Flow performance
  cv::Mat img_gray;
  if (img.channels() == 3) {
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
  } else {
    img_gray = img;
  }

  if (prev_image_.empty() || prev_keypoints_.size() < 100) {
    // Feature Detection
    detector_->detect(img_gray, prev_keypoints_);
    
    if (prev_keypoints_.empty()) {
      RCLCPP_WARN(get_logger(), "No features detected.");
      return;
    }

    prev_image_ = img_gray.clone();
    return;
  }

  std::vector<cv::Point2f> prev_points, curr_points;
  for (const auto & kp : prev_keypoints_) {
    prev_points.push_back(kp.pt);
  }

  std::vector<uchar> status;
  std::vector<float> error;

  // Calculate flow
  cv::calcOpticalFlowPyrLK(prev_image_, img_gray, prev_points, curr_points, status, error);

  // Filter good matches based on status and error
  std::vector<cv::Point2f> good_prev_points, good_curr_points;
  for (size_t i = 0; i < curr_points.size(); i++) {
    // Check if tracked successfully AND error is acceptable
    if (i < status.size() && status[i] && error[i] < 12.0) {
      good_prev_points.push_back(prev_points[i]);
      good_curr_points.push_back(curr_points[i]);
    }
  }

  // Visualization
  cv::Mat img_match;
  img.copyTo(img_match);
  for (size_t i = 0; i < good_curr_points.size(); i++) {
    cv::circle(img_match, good_curr_points[i], 2, cv::Scalar(0, 250, 0), 2);
    cv::line(img_match, good_prev_points[i], good_curr_points[i], cv::Scalar(0, 250, 0));
  }

  // Publish the matches image
  auto msg_matches = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", img_match).toImageMsg();
  msg_matches->header.frame_id = image_msg->header.frame_id;
  matches_pub_.publish(*msg_matches);

  cv::Mat R, t;
  estimatePose(good_curr_points, good_prev_points, R, t);
  RCLCPP_INFO_STREAM(get_logger(), "Estimated Rotation:\n" << R << "\nEstimated Translation:\n" << t);

  prev_image_ = img_gray.clone();
  prev_keypoints_.clear();
  for (const auto & pt : good_curr_points) {
    prev_keypoints_.push_back(cv::KeyPoint(pt, 1.0f));
  }
}

void OpticalFlow::estimatePose(
  const std::vector<cv::Point2f> & curr_points,
  const std::vector<cv::Point2f> & prev_points,
  cv::Mat & R, cv::Mat & t)
{
  if (curr_points.size() < 5) {
    RCLCPP_WARN(get_logger(), "Not enough points to estimate pose.");
    return;
  }

  // Calculate the Essential Matrix
  cv::Mat essential_matrix = cv::findEssentialMat(prev_points, curr_points, K_);
  if (essential_matrix.empty() || essential_matrix.rows != 3 || essential_matrix.cols != 3) {
    RCLCPP_WARN(get_logger(), "Failed to compute Essential Matrix.");
    return;
  }

  cv::recoverPose(essential_matrix, prev_points, curr_points, K_, R, t);
}
}  // namespace bumperbot_vision

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bumperbot_vision::OpticalFlow>("optical_flow"));
  rclcpp::shutdown();
  return 0;
}