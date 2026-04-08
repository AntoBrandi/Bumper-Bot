#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

class WebcamPublisher : public rclcpp::Node
{
public:
  WebcamPublisher()
  : Node("webcam_publisher")
  {
    // Initialize the publisher on the topic 'camera/image_raw'
    publisher_ = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

    // Open the default webcam (usually device 0)
    cap_.open(0);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Could not open video stream from webcam.");
    }

    // Set up a timer to capture and publish frames at approximately 30 Hz (33ms)
    timer_ = create_wall_timer(
      33ms, std::bind(&WebcamPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if (!cap_.isOpened()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Could not open video stream from webcam.");
      return;
    }

    cv::Mat frame;
    cap_ >> frame; // Read a new frame from the webcam

    // If the frame is empty, log a warning and skip publishing
    if (frame.empty()) {
      RCLCPP_WARN(get_logger(), "Captured empty frame, skipping.");
      return;
    }

    // Set up the ROS 2 message header
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = "camera_frame";

    // Use cv_bridge to convert the OpenCV Mat to a ROS 2 Image message
    // "bgr8" is the standard color encoding for OpenCV images
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
      header, "bgr8", frame).toImageMsg();

    // Publish the message
    publisher_->publish(*msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebcamPublisher>());
  rclcpp::shutdown();
  return 0;
}