#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np

import cv2
import message_filters

class OpticalFlow(Node):
    def __init__(self):
        super().__init__('optical_flow')

        self.prev_image = None
        self.prev_keypoints = None

        self.bridge = CvBridge()

        # Feature detection and matching
        self.detector = cv2.GFTTDetector_create(
            maxCorners=500, qualityLevel=0.01, minDistance=20.0)

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
            current_image_gray = cv2.cvtColor(current_image, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
        if self.prev_image is None or self.prev_points is None or len(self.prev_points) < 100:
            # Feature detection
            curr_keypoints = self.detector.detect(current_image_gray, None)
            
            if not curr_keypoints:
                self.get_logger().warn("No features detected to track.")
                return

            self.prev_points = np.array([kp.pt for kp in curr_keypoints], dtype=np.float32).reshape(-1, 1, 2)
            self.prev_image = current_image_gray.copy()
            return

        curr_points, status, error = cv2.calcOpticalFlowPyrLK(
            self.prev_image, current_image_gray, self.prev_points, None)
        
        # Filter good matches
        good_prev_points = []
        good_curr_points = []

        if curr_points is not None:
            for i, (p_curr, p_prev, s, e) in enumerate(zip(curr_points, self.prev_points, status, error)):
                if s[0] == 1 and e[0] < 12.0:
                    good_prev_points.append(p_prev[0])
                    good_curr_points.append(p_curr[0])

        img_match = current_image.copy()
        for pt_curr, pt_prev in zip(good_curr_points, good_prev_points):
            pt1 = (int(pt_curr[0]), int(pt_curr[1]))
            pt2 = (int(pt_prev[0]), int(pt_prev[1]))
            cv2.circle(img_match, pt1, 2, (0, 250, 0), 2)
            cv2.line(img_match, pt2, pt1, (0, 250, 0))

        # Publish Matches
        try:
            msg_matches = self.bridge.cv2_to_imgmsg(img_match, "bgr8")
            msg_matches.header.frame_id = image_msg.header.frame_id
            msg_matches.header.stamp = image_msg.header.stamp
            self.matches_pub.publish(msg_matches)
        except Exception as e:
            self.get_logger().error(f"Failed to publish matches: {e}")

        # Estimate Pose
        R, t = self.estimatePose(good_curr_points, good_prev_points)
        if R is not None and t is not None:
            self.get_logger().info(f"Estimated Rotation:\n{R}\nEstimated Translation:\n{t}")

        self.prev_image = current_image_gray.copy()
        if len(good_curr_points) > 0:
            self.prev_points = np.array(good_curr_points, dtype=np.float32).reshape(-1, 1, 2)
        else:
            self.prev_points = None

    def estimatePose(self, curr_points, prev_points):
        if len(curr_points) < 5:
            return None, None

        pts1 = np.array(prev_points, dtype=np.float32)
        pts2 = np.array(curr_points, dtype=np.float32)

        # Calculate the Essential Matrix
        essential_matrix, _ = cv2.findEssentialMat(pts1, pts2, self.K)
        if essential_matrix is None or essential_matrix.shape != (3, 3):
            return None, None
        
        # Recover Pose
        _, R, t, _ = cv2.recoverPose(essential_matrix, pts1, pts2, self.K)
        return R, t

def main(args=None):
    rclpy.init(args=args)
    optical_flow = OpticalFlow()
    rclpy.spin(optical_flow)
    optical_flow.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
