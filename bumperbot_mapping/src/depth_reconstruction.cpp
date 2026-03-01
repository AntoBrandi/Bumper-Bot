#include "bumperbot_mapping/depth_reconstruction.hpp"

#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"

#include "pcl_ros/transforms.hpp"
#include "pcl_conversions/pcl_conversions.h"

namespace bumperbot_mapping
{
DepthReconstruction::DepthReconstruction(const std::string & name)
: rclcpp::Node(name)
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void DepthReconstruction::initialize()
{
    pct_ = std::make_shared<point_cloud_transport::PointCloudTransport>(shared_from_this());
    cloud_sub_ = pct_->subscribe(
        "camera/points", 10,
        std::bind(&DepthReconstruction::cloudCallback, this, std::placeholders::_1));
    merged_cloud_pub_ = pct_->advertise("merged_cloud", 10);
}

void DepthReconstruction::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg)
{
    // Convert ROS PointCloud2 message to PCL format
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl::fromROSMsg(*cloud_msg, pcl_cloud);

    // Filter out points that are too far away
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(pcl_cloud));
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_ptr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.1, 5.0);
    pass.filter(pcl_cloud);

    // Transform the point cloud from the camera frame to the global frame
    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
    if (!pcl_ros::transformPointCloud(
        "odom", pcl_cloud, transformed_cloud, *tf_buffer_)) {
        RCLCPP_ERROR(get_logger(), "Failed to transform point cloud!");
        return;
    }

    // Accumulate the transformed point cloud into our global map
    if (merged_cloud_.empty()) {
        merged_cloud_ = transformed_cloud;
    } else {
        merged_cloud_ += transformed_cloud; 
    }

    // Downsample the merged pointcloud
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    float leaf = static_cast<float>(0.05);
    voxel_filter.setLeafSize(leaf, leaf, leaf);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(merged_cloud_));
    pcl::PointCloud<pcl::PointXYZRGB> filtered_merged;
    voxel_filter.setInputCloud(merged_ptr);
    voxel_filter.filter(filtered_merged);
    merged_cloud_ = filtered_merged;

    // Convert the merged PCL cloud back to a ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 merged_msg;
    pcl::toROSMsg(merged_cloud_, merged_msg);
    merged_msg.header.frame_id = "odom";
    merged_msg.header.stamp = now();
    merged_cloud_pub_.publish(merged_msg);
}
}  // namespace bumperbot_mapping


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bumperbot_mapping::DepthReconstruction>("depth_reconstruction");
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}