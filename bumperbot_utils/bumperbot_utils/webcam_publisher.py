#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        
        # Initialize the publisher on the topic 'camera/image_raw'
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        
        # Open the default webcam (usually device 0)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video stream from webcam.')
            
        # Initialize cv_bridge
        self.br = CvBridge()
        
        # Set up a timer to capture and publish frames at ~30 Hz (0.033 seconds)
        timer_period = 0.033 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warning('Captured empty frame, skipping.')
            return

        # Convert the OpenCV Mat to a ROS 2 Image message
        # "bgr8" is the standard color encoding for OpenCV images
        msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
        
        # Set up the ROS 2 message header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'

        # Publish the message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    
    try:
        rclpy.spin(webcam_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up resources
        webcam_publisher.cap.release()
        webcam_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()