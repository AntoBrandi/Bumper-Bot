#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np

import cv2
import message_filters

class FeatureDetector(Node):
    def __init__(self):
        super().__init__('feature_detector')

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
        # Camera intrinsic parameters
        self.K = np.array([
            [info_msg.k[0], info_msg.k[1], info_msg.k[2]],
            [info_msg.k[3], info_msg.k[4], info_msg.k[5]],
            [info_msg.k[6], info_msg.k[7], info_msg.k[8]]
        ], dtype=np.float64)

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

        R, t = self.estimatePose(keypoints, good_matches)
        self.get_logger().info(f"Estimated Rotation:\n{R}\nEstimated Translation:\n{t}")

        # Update previous frame data
        self.prev_image = current_image
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors

    def estimatePose(self, curr_keypoints, matches):
        points_1 = np.array([self.prev_keypoints[m.queryIdx].pt for m in matches], dtype=np.float32)
        points_2 = np.array([curr_keypoints[m.trainIdx].pt for m in matches], dtype=np.float32)
        
        # Calculate the Essential Matrix
        essential_matrix, _ = cv2.findEssentialMat(points_1, points_2, self.K)

        # Calculate the Homography Matrix
        homography_matrix, _ = cv2.findHomography(points_1, points_2, cv2.RANSAC, 3)
        _, R, t, _ = cv2.recoverPose(essential_matrix, points_1, points_2, self.K)
        return R, t

def main(args=None):
    rclpy.init(args=args)
    feature_detector = FeatureDetector()
    rclpy.spin(feature_detector)
    feature_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
