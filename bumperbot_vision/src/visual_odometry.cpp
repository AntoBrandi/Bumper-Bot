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

  cv::Mat T_rel_opencv = cv::Mat::eye(4, 4, CV_64F);
  cv::Mat img_match;

  if(use_direct_method_) {
    direct_method_->compute(img, img_match);
    T_rel_opencv = direct_method_->getTransform(K);
  } else {
    feature_method_->compute(img, img_match);
    T_rel_opencv = feature_method_->getTransform(K);
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

  cv::Mat t_cv = T_rel_opencv(cv::Rect(3, 0, 1, 3));
  cv::Mat T_rel_ros = cv::Mat::eye(4, 4, CV_64F);
  T_rel_ros.at<double>(0, 3) = t_cv.at<double>(2);
  T_rel_ros.at<double>(1, 3) = -t_cv.at<double>(0);
  T_rel_ros.at<double>(2, 3) = -t_cv.at<double>(1);

  double dt = (current_frame_time - last_frame_time_).seconds();
  calculateOdometry(T_rel_ros, dt);
  last_frame_time_ = current_frame_time;
  if(use_direct_method_) {
    direct_method_->update();
  } else {
    feature_method_->update();
  }
}

void VisualOdometry::calculateOdometry(const cv::Mat & T_rel, const double dt)
{
  if(dt <= 0){
    return;
  }

  cv::Mat T_move = T_rel.inv();
  cv::Mat R_move = T_move(cv::Rect(0, 0, 3, 3));
  double delta_yaw = atan2(R_move.at<double>(1, 0), R_move.at<double>(0, 0));
  
  cv::Mat T_rel_clean = cv::Mat::eye(4, 4, CV_64F);
  T_rel_clean.at<double>(0, 0) = cos(delta_yaw);
  T_rel_clean.at<double>(0, 1) = -sin(delta_yaw);
  T_rel_clean.at<double>(1, 0) = sin(delta_yaw);
  T_rel_clean.at<double>(1, 1) = cos(delta_yaw);

  T_rel_clean.at<double>(0, 3) = T_move.at<double>(0, 3);
  T_rel_clean.at<double>(1, 3) = T_move.at<double>(1, 3);
  T_rel_clean.at<double>(2, 3) = 0.0;

  world_pose_ = world_pose_ * T_rel_clean;

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "odom";
  odom.header.stamp = get_clock()->now();
  odom.child_frame_id = "base_link";

  // Linear Velocity (v = distance / time)
  // Note: These are unitless/scaled by 1.0 until scale is applied
  odom.twist.twist.linear.x = T_rel_clean.at<double>(0, 3) / dt;
  odom.twist.twist.linear.y = T_rel_clean.at<double>(1, 3) / dt;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.z = delta_yaw / dt;

  // --- Position ---
  odom.pose.pose.position.x = world_pose_.at<double>(0, 3);
  odom.pose.pose.position.y = world_pose_.at<double>(1, 3);
  odom.pose.pose.position.z = 0.0;

  // Extract Translation
  cv::Mat R_world = world_pose_(cv::Rect(0, 0, 3, 3));
  double global_yaw = atan2(R_world.at<double>(1, 0), R_world.at<double>(0, 0));

  // Convert Yaw to Quaternion: [x=0, y=0, z=sin(yaw/2), w=cos(yaw/2)]
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(global_yaw / 2.0);
  odom.pose.pose.orientation.w = cos(global_yaw / 2.0);

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
  curr_image_ = img.clone();
  curr_keypoints_.clear();
  detector_->detect(curr_image_, curr_keypoints_);
  curr_descriptor_ = cv::Mat(); // clear previous
  descriptor_->compute(curr_image_, curr_keypoints_, curr_descriptor_);

  if (prev_image_.empty()) {
    // Initialize if first frame
    update();
    return;
  }

  std::vector<cv::DMatch> matches;
  matcher_->match(prev_descriptor_, curr_descriptor_, matches);

  auto min_max = std::minmax_element(matches.begin(), matches.end(),
    [](const cv::DMatch & m1, const cv::DMatch & m2) { return m1.distance < m2.distance; });
  double min_dist = min_max.first->distance;

  good_matches_.clear();
  for(const auto& m : matches) {
    if(m.distance <= std::max(2 * min_dist, 30.0)) {
      good_matches_.push_back(m);
    }
  }

  cv::drawMatches(prev_image_, prev_keypoints_, curr_image_, curr_keypoints_, good_matches_, img_match);
}

cv::Mat FeatureMethod::getTransform(const cv::Mat & K)
{
  if (good_matches_.size() < 10) return cv::Mat::eye(4, 4, CV_64F);

  std::vector<cv::Point2f> p1, p2;
  double total_dist = 0;
  for (const auto & m : good_matches_) {
    p1.push_back(prev_keypoints_[m.queryIdx].pt); 
    p2.push_back(curr_keypoints_[m.trainIdx].pt);
    total_dist += cv::norm(p1.back() - p2.back());
  }

  double avg_dist = total_dist / good_matches_.size();
  if (avg_dist < 1.0) return cv::Mat::eye(4, 4, CV_64F);

  cv::Mat R, t, mask;
  cv::Mat E = cv::findEssentialMat(p1, p2, K, cv::RANSAC, 0.999, 1.0, mask);

  cv::recoverPose(E, p1, p2, K, R, t, mask);

  cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
  R.copyTo(T(cv::Rect(0, 0, 3, 3)));
  t.copyTo(T(cv::Rect(3, 0, 1, 3)));
  return T;
}

void FeatureMethod::update()
{
  prev_image_ = curr_image_.clone();
  prev_keypoints_ = curr_keypoints_;
  prev_descriptor_ = curr_descriptor_.clone();
}

DirectMethod::DirectMethod()
{
  detector_ = cv::GFTTDetector::create(500, 0.01, 20);
}

void DirectMethod::compute(const cv::Mat & img, cv::Mat & img_match)
{
  curr_image_ = img.clone();

  if (prev_image_.empty()) {
    update();
    return;
  }

  pts_prev_.clear();
  for (auto & kp : prev_keypoints_) pts_prev_.push_back(kp.pt);

  pts_curr_.clear();
  status_.clear();
  std::vector<float> error;

  // Calculate flow
  if (!pts_prev_.empty()) {
      cv::calcOpticalFlowPyrLK(prev_image_, curr_image_, pts_prev_, pts_curr_, status_, error);
  }

  // Visualization
  curr_image_.copyTo(img_match);
  for (size_t i = 0; i < pts_curr_.size(); i++) {
    if (i < status_.size() && status_[i]) {
      cv::circle(img_match, pts_curr_[i], 2, cv::Scalar(0, 250, 0), 2);
      cv::line(img_match, pts_prev_[i], pts_curr_[i], cv::Scalar(0, 250, 0));
    }
  }
}

void DirectMethod::update()
{
  prev_image_ = curr_image_.clone();
  detector_->detect(prev_image_, prev_keypoints_);
}

cv::Mat DirectMethod::getTransform(const cv::Mat & K) {
  std::vector<cv::Point2f> valid_prev, valid_curr;
  double total_dist = 0;

  // 1. Filter points based on the status from calcOpticalFlowPyrLK
  for (size_t i = 0; i < status_.size(); i++) {
    if (status_[i]) {
      valid_prev.push_back(pts_prev_[i]);
      valid_curr.push_back(pts_curr_[i]);
      total_dist += cv::norm(pts_curr_[i] - pts_prev_[i]);
    }
  }

  // 2. Stability Check: Need at least 8 points for the Essential Matrix
  if (valid_prev.size() < 10) {
    return cv::Mat::eye(4, 4, CV_64F);
  }

  // 3. Motion Threshold: Prevent "Ghost" movement when stopped
  double avg_dist = total_dist / valid_prev.size();
  if (avg_dist < 1.0) { // Slightly lower threshold than features as flow is smoother
    return cv::Mat::eye(4, 4, CV_64F);
  }

  // 4. Robust Pose Recovery using RANSAC
  cv::Mat R, t, mask;
  // findEssentialMat with RANSAC removes points that don't follow the "camera motion"
  cv::Mat E = cv::findEssentialMat(valid_prev, valid_curr, K, cv::RANSAC, 0.999, 1.0, mask);
  
  // recoverPose uses the RANSAC mask to calculate R and t only from valid "inliers"
  cv::recoverPose(E, valid_prev, valid_curr, K, R, t, mask);

  // 5. Build the 4x4 matrix
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