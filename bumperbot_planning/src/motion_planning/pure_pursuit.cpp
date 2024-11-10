#include "bumperbot_planning/motion_planning/pure_pursuit.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bumperbot_planning
{
PurePursuit::PurePursuit() : Node("pure_pursuit_node"),
    look_ahead_distance_(0.5), goal_tolerance_(0.1), max_linear_velocity_(0.3)
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    declare_parameter<double>("look_ahead_distance", look_ahead_distance_);
    declare_parameter<double>("goal_tolerance", goal_tolerance_);
    declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
    look_ahead_distance_ = get_parameter("look_ahead_distance").as_double();
    goal_tolerance_ = get_parameter("goal_tolerance").as_double();
    max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/a_star/path", 10, std::bind(&PurePursuit::pathCallback, this, std::placeholders::_1));
        
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    carrot_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pure_pursuit/carrot", 10);
}

void PurePursuit::pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg)
{
    while(rclcpp::ok()) {
        // Get the robot's current pose in the path frame
        try {
            map_to_base_ts_ = tf_buffer_->lookupTransform(
                path_msg->header.frame_id, "base_footprint", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Could not transform: %s", ex.what());
            return;
        }

        geometry_msgs::msg::PoseStamped carrot_pose;
        getCarrotPose(path_msg, carrot_pose);

        double dx = carrot_pose.pose.position.x - map_to_base_ts_.transform.translation.x;
        double dy = carrot_pose.pose.position.y - map_to_base_ts_.transform.translation.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        if(distance <= goal_tolerance_){
            RCLCPP_INFO(get_logger(), "Goal Reached!");
            break;
        }

        // Transform the look-ahead point into the robot's frame
        tf2::Transform map_to_base_tf, map_to_carrot_pose_tf;
        tf2::fromMsg(map_to_base_ts_.transform, map_to_base_tf);
        tf2::fromMsg(carrot_pose.pose, map_to_carrot_pose_tf);

        geometry_msgs::msg::PoseStamped carrot_pose_robot_frame;
        carrot_pose_robot_frame.header.frame_id = "base_footprint";
        tf2::toMsg(map_to_base_tf.inverse() * map_to_carrot_pose_tf, carrot_pose_robot_frame.pose);
        carrot_pub_->publish(carrot_pose_robot_frame);
        
        // Calculate the curvature to the look-ahead point
        double curvature = getCurvature(carrot_pose_robot_frame.pose);
        
        // Create and publish the velocity command
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = max_linear_velocity_;
        cmd_vel.angular.z = curvature * max_linear_velocity_;
        cmd_pub_->publish(cmd_vel);
    }
}

void PurePursuit::getCarrotPose(const nav_msgs::msg::Path::SharedPtr path, geometry_msgs::msg::PoseStamped & carrot_pose)
{
    // Find the look-ahead point on the path
    for(const auto & pose : path->poses){
        carrot_pose = pose;
        double dx = pose.pose.position.x - map_to_base_ts_.transform.translation.x;
        double dy = pose.pose.position.y - map_to_base_ts_.transform.translation.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        if(distance < look_ahead_distance_){
            return;
        }
    }
}

double PurePursuit::getCurvature(const geometry_msgs::msg::Pose & carrot_pose)
{
    const double carrot_dist2 =
    (carrot_pose.position.x * carrot_pose.position.x) +
    (carrot_pose.position.y * carrot_pose.position.y);
    
    // Find curvature of circle (k = 1 / R)
    if (carrot_dist2 > 0.001) {
      return 2.0 * carrot_pose.position.y / carrot_dist2;
    } else {
      return 0.0;
    }
}
}  // namespace bumperbot_planning

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_planning::PurePursuit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}