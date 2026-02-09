#include "bumperbot_vision/visual_odometry.hpp"

#include "cv_bridge/cv_bridge.hpp"

namespace bumperbot_vision
{
VisualOdometry::VisualOdometry(const std::string & name)
: Node(name), use_direct_method_{false}
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

  declare_parameter<bool>("use_direct_method", use_direct_method_);
  use_direct_method_ = get_parameter("use_direct_method").as_bool();

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("vo_odom", 10);
  world_pose_ = cv::Mat::eye(4, 4, CV_64F);

  if (use_direct_method_){
    direct_method_ = std::make_unique<DirectMethod>();
  } else {
    feature_method_ = std::make_unique<FeatureMethod>();
  }
}

void VisualOdometry::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  rclcpp::Time current_frame_time = image_msg->header.stamp;
  cv::Mat img = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
  cv::Mat K = (cv::Mat_<double>(3, 3) << 
        info_msg->k[0], info_msg->k[1], info_msg->k[2],
        info_msg->k[3], info_msg->k[4], info_msg->k[5],
        info_msg->k[6], info_msg->k[7], info_msg->k[8]);

  cv::Mat T_rel = cv::Mat::eye(4, 4, CV_64F);
  cv::Mat img_match;

  if(use_direct_method_) {
    direct_method_->compute(img, img_match);
    T_rel = direct_method_->getTransform(K);
  } else {
    feature_method_->compute(img, img_match);
    T_rel = feature_method_->getTransform(K);
  }

  // Publish the matches image
  auto msg_matches = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", img_match).toImageMsg();
  msg_matches->header.frame_id = image_msg->header.frame_id;
  matches_pub_.publish(*msg_matches);

  if (first_frame_) {
    last_frame_time_ = current_frame_time;
    first_frame_ = false;
    return; 
  }

  double dt = (current_frame_time - last_frame_time_).seconds();
  calculateOdometry(T_rel, dt);
  last_frame_time_ = current_frame_time;
}

void VisualOdometry::calculateOdometry(const cv::Mat & T_rel, const double dt)
{
  if(dt <= 0){
    return;
  }

  world_pose_ = world_pose_ * T_rel;

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  // T_rel is the transform from current to previous frame
  // Translation components:
  double dx = T_rel.at<double>(0, 3);
  double dy = T_rel.at<double>(1, 3);
  double dz = T_rel.at<double>(2, 3);

  // Linear Velocity (v = distance / time)
  // Note: These are unitless/scaled by 1.0 until scale is applied
  odom.twist.twist.linear.x = dx / dt;
  odom.twist.twist.linear.y = dy / dt;
  odom.twist.twist.linear.z = dz / dt;

  // Angular Velocity (Simplified from Rotation Matrix)
  // Using small angle approximation or extracting Euler angles
  cv::Mat R_rel = T_rel(cv::Rect(0, 0, 3, 3));
  double roll = atan2(R_rel.at<double>(2,1), R_rel.at<double>(2,2));
  double pitch = atan2(-R_rel.at<double>(2,0), sqrt(pow(R_rel.at<double>(2,1),2) + pow(R_rel.at<double>(2,2),2)));
  double yaw = atan2(R_rel.at<double>(1,0), R_rel.at<double>(0,0));

  odom.twist.twist.angular.x = roll / dt;
  odom.twist.twist.angular.y = pitch / dt;
  odom.twist.twist.angular.z = yaw / dt;

  // Extract Translation
  odom.pose.pose.position.x = world_pose_.at<double>(0, 3);
  odom.pose.pose.position.y = world_pose_.at<double>(1, 3);
  odom.pose.pose.position.z = world_pose_.at<double>(2, 3);

  // Extract Rotation Matrix to Quaternion
  cv::Mat R = world_pose_(cv::Rect(0, 0, 3, 3));
  double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
    
  // Simple conversion logic (or use tf2::Quaternion)
  double qw = sqrt(1.0 + trace) / 2.0;
  odom.pose.pose.orientation.w = qw;
  odom.pose.pose.orientation.x = (R.at<double>(2,1) - R.at<double>(1,2)) / (4.0 * qw);
  odom.pose.pose.orientation.y = (R.at<double>(0,2) - R.at<double>(2,0)) / (4.0 * qw);
  odom.pose.pose.orientation.z = (R.at<double>(1,0) - R.at<double>(0,1)) / (4.0 * qw);

  odom_pub_->publish(odom);
}

FeatureMethod::FeatureMethod()
{
  detector_ = cv::ORB::create();
  descriptor_ = cv::ORB::create();
  matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

void FeatureMethod::compute(const cv::Mat & img, cv::Mat & img_match)
{
  curr_keypoints_.clear(); // New member variable
  detector_->detect(img, curr_keypoints_);
  cv::Mat descriptors;
  descriptor_->compute(img, curr_keypoints_, descriptors);

  if (prev_image_.empty()) {
    prev_image_ = img.clone();
    prev_keypoints_ = curr_keypoints_;
    prev_descriptor_ = descriptors;
    return;
  }

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
  good_matches_.clear();
  for(const auto& m : matches) {
    if(m.distance <= std::max(2 * min_dist, 30.0)) {
      good_matches_.push_back(m);
    }
  }

  // Draw the results
  cv::drawMatches(
    prev_image_, prev_keypoints_, img, curr_keypoints_, good_matches_, img_match);

  // Update previous frame data
  prev_image_ = img.clone();
  prev_keypoints_ = curr_keypoints_;
  prev_descriptor_ = descriptors;
}

cv::Mat FeatureMethod::getTransform(const cv::Mat & K)
{
  if (good_matches_.size() < 8) return cv::Mat::eye(4, 4, CV_64F);

  std::vector<cv::Point2f> p1, p2;
  for(const auto & m : good_matches_) {
    p1.push_back(prev_keypoints_[m.queryIdx].pt);
    p2.push_back(curr_keypoints_[m.trainIdx].pt);
  }

  cv::Mat R, t, mask;
  cv::Mat E = cv::findEssentialMat(p1, p2, K, cv::RANSAC, 0.999, 1.0, mask);
  cv::recoverPose(E, p1, p2, K, R, t, mask);

  cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
  R.copyTo(T(cv::Rect(0, 0, 3, 3)));
  t.copyTo(T(cv::Rect(3, 0, 1, 3)));
  return T;
}

DirectMethod::DirectMethod()
{
  detector_ = cv::GFTTDetector::create(500, 0.01, 20);
}

void DirectMethod::compute(const cv::Mat & img, cv::Mat & img_match)
{
  if (prev_image_.empty()) {
    prev_image_ = img.clone();
    detector_->detect(prev_image_, prev_keypoints_);
    return;
  }

  pts_prev_.clear();
  for (auto & kp : prev_keypoints_) pts_prev_.push_back(kp.pt);

  pts_curr_.clear();
  status_.clear();
  std::vector<float> error;

  cv::calcOpticalFlowPyrLK(prev_image_, img, pts_prev_, pts_curr_, status_, error);

  img.copyTo(img_match);
  for (size_t i = 0; i < pts_curr_.size(); i++) {
    if (status_[i]) {
      cv::circle(img_match, pts_curr_[i], 2, cv::Scalar(0, 250, 0), 2);
      cv::line(img_match, pts_prev_[i], pts_curr_[i], cv::Scalar(0, 250, 0));
    }
  }

  // Update previous frame data
  prev_image_ = img.clone();
  detector_->detect(prev_image_, prev_keypoints_);
}

cv::Mat DirectMethod::getTransform(const cv::Mat & K) {
  std::vector<cv::Point2f> valid_prev, valid_curr;
  for (size_t i = 0; i < status_.size(); i++) {
    if (status_[i]) {
      valid_prev.push_back(pts_prev_[i]);
      valid_curr.push_back(pts_curr_[i]);
    }
  }

  if (valid_prev.size() < 8) return cv::Mat::eye(4, 4, CV_64F);

  cv::Mat R, t, mask;
  cv::Mat E = cv::findEssentialMat(valid_prev, valid_curr, K, cv::RANSAC, 0.999, 1.0, mask);
  cv::recoverPose(E, valid_prev, valid_curr, K, R, t, mask);

  cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
  R.copyTo(T(cv::Rect(0, 0, 3, 3)));
  t.copyTo(T(cv::Rect(3, 0, 1, 3)));
  return T;
}
}  // namespace bumperbot_vision

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bumperbot_vision::VisualOdometry>("visual_odometry"));
  rclcpp::shutdown();
  return 0;
}