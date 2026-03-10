#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

import cv2
import numpy as np

class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')
        
        # 1. Declare Node Parameters with Gazebo defaults
        self.declare_parameter('lower_hsv', [79, 50, 50])
        self.declare_parameter('upper_hsv', [99, 255, 255])
        self.declare_parameter('ball_radius', 0.31)
        self.lower_hsv = self.get_parameter('lower_hsv').value
        self.upper_hsv = self.get_parameter('upper_hsv').value
        self.ball_radius = self.get_parameter('ball_radius').value
        
        self.bridge = CvBridge()
        
        self.image_sub = message_filters.Subscriber(
            self, Image, 'left_camera/image_rect', qos_profile=qos_profile_sensor_data)
        self.info_sub = message_filters.Subscriber(
            self, CameraInfo, 'left_camera/camera_info', qos_profile=qos_profile_sensor_data)
            
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)
        
        self.detection_pub = self.create_publisher(
            Image, 'left_camera/ball_detection', qos_profile_sensor_data)
        self.ball_pose_pub = self.create_publisher(
            PoseStamped, 'goal_update', 10)

    def image_callback(self, image_msg, info_msg):
        
        # Convert ROS Image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return
            
        # Convert to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Create NumPy arrays for the color bounds
        lower_bound = np.array(self.lower_hsv)
        upper_bound = np.array(self.upper_hsv)
        
        # Create the mask
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        
        # Morphological Operations to clean up noise and shadows
        kernel = np.ones((5, 5), np.uint8)
        # OPENING: Remove small background noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        # CLOSING: Fill in holes and gaps caused by shadows
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=5)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Get the minimum enclosing circle
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            
            if radius > 10.0:
                # Draw the bounding box and center
                x_int, y_int = int(x), int(y)
                x_rect, y_rect, w, h = cv2.boundingRect(largest_contour)
                
                cv2.rectangle(cv_image, (x_rect, y_rect), (x_rect + w, y_rect + h), (0, 255, 0), 2)
                cv2.circle(cv_image, (x_int, y_int), 3, (0, 0, 255), -1)
                
                # Pose Estimation using CameraInfo
                fx = info_msg.k[0]
                cx = info_msg.k[2]
                fy = info_msg.k[4]
                cy = info_msg.k[5]
                
                if fx > 0 and fy > 0:
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = image_msg.header.stamp
                    pose_msg.header.frame_id = info_msg.header.frame_id
                    
                    # Estimate Z (Depth) and X, Y
                    estimated_z = (fx * self.ball_radius) / radius
                    estimated_x = (x - cx) * estimated_z / fx
                    estimated_y = (y - cy) * estimated_z / fy
                    
                    pose_msg.pose.position.x = float(estimated_x)
                    pose_msg.pose.position.y = float(estimated_y)
                    pose_msg.pose.position.z = float(estimated_z)
                    pose_msg.pose.orientation.w = 1.0
                    
                    self.ball_pose_pub.publish(pose_msg)
                    
        # Convert annotated image back to ROS and publish
        detection_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        detection_msg.header = image_msg.header
        self.detection_pub.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()