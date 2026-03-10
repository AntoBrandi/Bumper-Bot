#include "bumperbot_navigation/ball_follower.hpp"

#include "cv_bridge/cv_bridge.hpp"

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

namespace bumperbot_navigation
{
BallFollower::BallFollower(const std::string & name) : Node(name)
{
  declare_parameter("lower_hsv", std::vector<int64_t>{79, 50, 50});
  declare_parameter("upper_hsv", std::vector<int64_t>{99, 255, 255});
  declare_parameter("ball_radius", 0.31);
  get_parameter("lower_hsv", lower_hsv_);
  get_parameter("upper_hsv", upper_hsv_);
  get_parameter("ball_radius", ball_radius_);

  approximate_sync_ = std::make_shared<ApproximateSync>(
    ApproximatePolicy(10), image_sub_, info_sub_);
  approximate_sync_->registerCallback(
    std::bind(&BallFollower::imageCallback, this, _1, _2));
  image_transport::TransportHints hints{this};
  const auto sensor_data_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();

  image_sub_.subscribe(
    this, "left_camera/image_rect", hints.getTransport(), sensor_data_qos);
  info_sub_.subscribe(this, "left_camera/camera_info", sensor_data_qos);

  detection_pub_ =
    image_transport::create_publisher(this, "left_camera/ball_detection", rmw_qos_profile_sensor_data);
  ball_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_update", 10);
}

void BallFollower::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  
  cv::Mat image = cv_ptr->image;
  cv::Mat hsv, mask;

  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

  cv::Scalar lower_bound(lower_hsv_[0], lower_hsv_[1], lower_hsv_[2]);
  cv::Scalar upper_bound(upper_hsv_[0], upper_hsv_[1], upper_hsv_[2]);
  cv::inRange(hsv, lower_bound, upper_bound, mask);

  cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (!contours.empty()) {
    auto largest_contour = std::max_element(contours.begin(), contours.end(),
      [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
        return cv::contourArea(a) < cv::contourArea(b);
      });

    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(*largest_contour, center, radius);

    if (radius > 10.0) {
      cv::Rect bbox = cv::boundingRect(*largest_contour);
      cv::rectangle(image, bbox, cv::Scalar(0, 255, 0), 2);
      cv::circle(image, center, 3, cv::Scalar(0, 0, 255), -1);

      double fx = info_msg->k[0];
      double cx = info_msg->k[2];
      double fy = info_msg->k[4];
      double cy = info_msg->k[5];

      if (fx > 0 && fy > 0) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = image_msg->header.stamp;
        pose_msg.header.frame_id = info_msg->header.frame_id;

        double estimated_z = (fx * ball_radius_) / radius;
        double estimated_x = (center.x - cx) * estimated_z / fx;
        double estimated_y = (center.y - cy) * estimated_z / fy;

        pose_msg.pose.position.x = estimated_x;
        pose_msg.pose.position.y = estimated_y;
        pose_msg.pose.position.z = estimated_z;
        pose_msg.pose.orientation.w = 1.0; 

        ball_pose_pub_->publish(pose_msg);
      }
    }
  }

  sensor_msgs::msg::Image::SharedPtr detection_msg = cv_bridge::CvImage(
    image_msg->header, sensor_msgs::image_encodings::BGR8, image).toImageMsg();
  detection_pub_.publish(*detection_msg);
}
}  // namespace bumperbot_navigation

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bumperbot_navigation::BallFollower>("ball_follower"));
  rclcpp::shutdown();
  return 0;
}