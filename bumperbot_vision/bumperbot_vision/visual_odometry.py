#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import cv2
import message_filters

class VisualOdometry(Node):
    def __init__(self):
        super().__init__('visual_odometry')

        self.prev_image = None
        self.prev_keypoints = None
        self.prev_descriptors = None

        self.bridge = CvBridge()

        # Feature detection and matching
        self.detector = cv2.ORB_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

        # Subscribers
        image_sub = message_filters.Subscriber(
            self, Image, "left_camera/image_raw", qos_profile=qos_profile_sensor_data)
        info_sub = message_filters.Subscriber(
            self, CameraInfo, "left_camera/camera_info", qos_profile=qos_profile_sensor_data)

        self.approximate_sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, info_sub], queue_size=10, slop=0.2)
        self.approximate_sync.registerCallback(self.image_callback)

        self.matches_pub = self.create_publisher(
            Image, "matches", qos_profile_sensor_data)

    def image_callback(self, image_msg, info_msg):
        # Convert ROS Image to OpenCV
        try:
            current_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Feature detection
        keypoints = self.detector.detect(current_image, None)
        
        # Compute descriptors
        keypoints, descriptors = self.detector.compute(current_image, keypoints)

        if self.prev_image is None:
            self.prev_image = current_image
            self.prev_keypoints = keypoints
            self.prev_descriptors = descriptors
            return

        if descriptors is None or len(descriptors) == 0:
             return

        # Feature matching
        matches = self.matcher.match(self.prev_descriptors, descriptors)

        # Sort matches by distance
        matches = sorted(matches, key=lambda x: x.distance)

        if not matches:
            return

        min_dist = matches[0].distance
        
        # Filter good matches
        good_matches = []
        for match in matches:
            if match.distance <= max(2 * min_dist, 30.0):
                good_matches.append(match)

        # Draw matches
        img_match = cv2.drawMatches(
            self.prev_image, self.prev_keypoints,
            current_image, keypoints,
            good_matches, None
        )

        # Publish matches image
        try:
            msg_matches = self.bridge.cv2_to_imgmsg(img_match, "bgr8")
            msg_matches.header.frame_id = image_msg.header.frame_id
            self.matches_pub.publish(msg_matches)
        except Exception as e:
             self.get_logger().error(f"Failed to publish matches: {e}")

        # Update previous frame data
        self.prev_image = current_image
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors

def main(args=None):
    rclpy.init(args=args)
    visual_odometry = VisualOdometry()
    rclpy.spin(visual_odometry)
    visual_odometry.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
