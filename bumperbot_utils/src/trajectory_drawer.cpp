#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

class TrajectoryDrawer : public rclcpp::Node
{
public:
  TrajectoryDrawer() : Node("trajectory_drawer")
  {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "bumperbot_controller/odom", 10, std::bind(&TrajectoryDrawer::odometryCallback, this, _1));
    trajectory_pub_ = create_publisher<nav_msgs::msg::Path>("bumperbot_controller/trajectory", 10);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  nav_msgs::msg::Path trajectory_;

  void odometryCallback(const nav_msgs::msg::Odometry &msg)
  {
    trajectory_.header.frame_id = msg.header.frame_id;
    geometry_msgs::msg::PoseStamped curr_pose;
    curr_pose.header.frame_id = msg.header.frame_id;
    curr_pose.header.stamp = msg.header.stamp;
    curr_pose.pose = msg.pose.pose;
    trajectory_.poses.push_back(curr_pose);

    trajectory_pub_->publish(trajectory_);
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryDrawer>());
  rclcpp::shutdown();
  return 0;
}