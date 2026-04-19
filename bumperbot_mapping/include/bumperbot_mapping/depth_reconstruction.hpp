#ifndef DEPTH_RECONSTRUCTION_HPP
#define DEPTH_RECONSTRUCTION_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "octomap_msgs/msg/octomap.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "point_cloud_transport/point_cloud_transport.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "octomap/ColorOcTree.h"

namespace bumperbot_mapping
{
class DepthReconstruction : public rclcpp::Node
{
public:
    DepthReconstruction(const std::string & name);

    void initialize();

private:
    std::shared_ptr<point_cloud_transport::PointCloudTransport> pct_;
    point_cloud_transport::Subscriber cloud_sub_;
    point_cloud_transport::Publisher merged_cloud_pub_;

    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    pcl::PointCloud<pcl::PointXYZRGB> merged_cloud_;

    std::shared_ptr<octomap::ColorOcTree> octree_;

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);
};
}

#endif  // DEPTH_RECONSTRUCTION_HPP