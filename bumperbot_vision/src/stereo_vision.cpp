#include "bumperbot_vision/stereo_vision.hpp"

#include "cv_bridge/cv_bridge.hpp"

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace bumperbot_vision
{
StereoVision::StereoVision(const std::string & name) : Node(name)
{
  approximate_sync_ = std::make_shared<ApproximateSync>(
    ApproximatePolicy(10), left_image_sub_, left_info_sub_, right_image_sub_, right_info_sub_);
  approximate_sync_->registerCallback(
    std::bind(&StereoVision::imageCallback, this, _1, _2, _3, _4));
  image_transport::TransportHints hints{this};
  const auto sensor_data_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();

  left_image_sub_.subscribe(
    this, "left_camera/image_rect", hints.getTransport(), sensor_data_qos);
  left_info_sub_.subscribe(this, "left_camera/camera_info", sensor_data_qos);
  right_image_sub_.subscribe(
    this, "right_camera/image_rect", hints.getTransport(), sensor_data_qos);
  right_info_sub_.subscribe(this, "right_camera/camera_info", sensor_data_qos);

  disparity_image_pub_ =
    image_transport::create_publisher(this, "disparity", rmw_qos_profile_sensor_data);

  // Declare parameters
  sgbm_params_.min_disparity = declare_parameter("min_disparity", 0);
  sgbm_params_.num_disparities = declare_parameter("num_disparities", 96);
  sgbm_params_.block_size = declare_parameter("block_size", 9);
  sgbm_params_.p1 = declare_parameter("p1", 8 * 9 * 9);
  sgbm_params_.p2 = declare_parameter("p2", 32 * 9 * 9);
  sgbm_params_.disp12_max_diff = declare_parameter("disp12_max_diff", 1);
  sgbm_params_.pre_filter_cap = declare_parameter("pre_filter_cap", 63);
  sgbm_params_.uniqueness_ratio = declare_parameter("uniqueness_ratio", 10);
  sgbm_params_.speckle_window_size = declare_parameter("speckle_window_size", 100);
  sgbm_params_.speckle_range = declare_parameter("speckle_range", 32);
  sgbm_params_.mode = declare_parameter("mode", static_cast<int>(cv::StereoSGBM::MODE_SGBM));
  baseline_ = declare_parameter("baseline", 0.06);

  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&StereoVision::onSetParameters, this, _1));
}

rcl_interfaces::msg::SetParametersResult StereoVision::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : parameters) {
    if (param.get_name() == "min_disparity") {
      sgbm_params_.min_disparity = param.as_int();
    } else if (param.get_name() == "num_disparities") {
      sgbm_params_.num_disparities = param.as_int();
    } else if (param.get_name() == "block_size") {
      sgbm_params_.block_size = param.as_int();
    } else if (param.get_name() == "p1") {
      sgbm_params_.p1 = param.as_int();
    } else if (param.get_name() == "p2") {
      sgbm_params_.p2 = param.as_int();
    } else if (param.get_name() == "disp12_max_diff") {
      sgbm_params_.disp12_max_diff = param.as_int();
    } else if (param.get_name() == "pre_filter_cap") {
      sgbm_params_.pre_filter_cap = param.as_int();
    } else if (param.get_name() == "uniqueness_ratio") {
      sgbm_params_.uniqueness_ratio = param.as_int();
    } else if (param.get_name() == "speckle_window_size") {
      sgbm_params_.speckle_window_size = param.as_int();
    } else if (param.get_name() == "speckle_range") {
      sgbm_params_.speckle_range = param.as_int();
    } else if (param.get_name() == "mode") {
      sgbm_params_.mode = param.as_int();
    } else if (param.get_name() == "baseline") {
      baseline_ = param.as_double();
    }
  }
  return result;
}

void StereoVision::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & left_image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & left_info_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & right_image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & /*right_info_msg*/)
{
  cv::Mat left = cv_bridge::toCvCopy(left_image_msg, left_image_msg->encoding)->image;
  cv::Mat right = cv_bridge::toCvCopy(right_image_msg, right_image_msg->encoding)->image;
  cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
    sgbm_params_.min_disparity, sgbm_params_.num_disparities, sgbm_params_.block_size,
    sgbm_params_.p1, sgbm_params_.p2, sgbm_params_.disp12_max_diff, sgbm_params_.pre_filter_cap,
    sgbm_params_.uniqueness_ratio, sgbm_params_.speckle_window_size, sgbm_params_.speckle_range,
    sgbm_params_.mode);
  cv::Mat disparity_sgbm, disparity;
  sgbm->compute(left, right, disparity_sgbm);
  disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0);
  cv::Mat normalized_disparity;
  cv::normalize(disparity_sgbm, normalized_disparity, 0, 255, cv::NORM_MINMAX, CV_8U);

  auto disparity_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", normalized_disparity).toImageMsg();
  disparity_msg->header = left_image_msg->header;
  disparity_image_pub_.publish(disparity_msg);

  // Generate PointCloud
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pointcloud_msg.header = left_image_msg->header;
  pointcloud_msg.height = 1;
  pointcloud_msg.width = left.rows * left.cols;
  pointcloud_msg.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
  modifier.setPointCloud2Fields(
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::FLOAT32);

  sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(pointcloud_msg, "intensity");

  for (int v = 0; v < left.rows; v++) {
    for (int u = 0; u < left.cols; u++) {
      if (disparity.at<float>(v, u) <= sgbm_params_.min_disparity ||
          disparity.at<float>(v, u) >= sgbm_params_.min_disparity + sgbm_params_.num_disparities) {
        continue;
      }
      double x = (u - left_info_msg->k[2]) / left_info_msg->k[0];
      double y = (v - left_info_msg->k[5]) / left_info_msg->k[4];
      double depth = left_info_msg->k[0] * baseline_ / (disparity.at<float>(v, u));
      *iter_x = x * depth;
      *iter_y = y * depth;
      *iter_z = depth;
      *iter_intensity = left.at<uchar>(v, u) / 255.0;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }
  }

  if(!pct_){
    pct_ = std::make_unique<point_cloud_transport::PointCloudTransport>(shared_from_this());
    point_cloud_pub_ = pct_->advertise("point_cloud", rmw_qos_profile_sensor_data);
  }
  point_cloud_pub_.publish(pointcloud_msg);
}
}  // namespace bumperbot_vision

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bumperbot_vision::StereoVision>("stereo_vision"));
  rclcpp::shutdown();
  return 0;
}