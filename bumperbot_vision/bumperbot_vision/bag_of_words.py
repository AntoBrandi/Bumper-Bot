#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped

from std_srvs.srv import Trigger
from bumperbot_msgs.srv import SetString

from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_ros import TransformException


class BagOfWords(Node):
    def __init__(self, name="bag_of_words"):
        super().__init__(name)

        self.is_training = True
        self.is_trained = False

        self.declare_parameter("training_frequency", 10.0)
        self.declare_parameter("vocabulary_size", 100)
        
        self.training_frequency = self.get_parameter("training_frequency").value
        self.vocabulary_size = self.get_parameter("vocabulary_size").value
        self.last_train_time = self.get_clock().now()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transform_broadcaster = TransformBroadcaster(self)

        self.bridge = CvBridge()

        self.image_sub = message_filters.Subscriber(self, Image, "left_camera/image_raw")
        self.info_sub = message_filters.Subscriber(self, CameraInfo, "left_camera/camera_info")
        
        self.approx_sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub], queue_size=10, slop=0.1)
        self.approx_sync.registerCallback(self.image_callback)

        self.detector = cv2.ORB_create()
        
        # We must use KDTree (algorithm=1) because KMeans vocabulary is float32
        index_params = dict(algorithm=1, trees=5)
        search_params = dict(checks=50)
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)
        
        # Data storage
        self.training_descriptors = [] 
        self.stored_poses = []
        self.database_histograms = []
        self.vocabulary = None

        self.train_mode_srv = self.create_service(Trigger, f"{name}/train_mode", self.train_mode_cb)
        self.test_mode_srv = self.create_service(Trigger, f"{name}/test_mode", self.test_mode_cb)
        self.save_srv = self.create_service(SetString, f"{name}/save", self.save_cb)
        self.load_srv = self.create_service(SetString, f"{name}/load", self.load_cb)


    def train_mode_cb(self, request, response):
        self.is_training = True
        response.success = True
        response.message = "Switched to train mode"
        return response


    def test_mode_cb(self, request, response):
        self.is_training = False
        response.success = True
        response.message = "Switched to test mode"
        return response


    def _compute_histogram(self, descriptors):
        """Manually matches descriptors to the vocabulary and builds a normalized histogram."""
        if self.vocabulary is None:
            return None
            
        # ORB outputs uint8, but vocabulary from KMeans is float32. We must cast to match.
        desc_float = np.float32(descriptors)
        
        # Match each descriptor to the nearest cluster center (visual word)
        matches = self.matcher.match(desc_float, self.vocabulary)
        
        # Build the histogram
        hist = np.zeros(self.vocabulary_size, dtype=np.float32)
        for m in matches:
            hist[m.trainIdx] += 1.0
            
        # Normalize
        hist_sum = np.sum(hist)
        if hist_sum > 0:
            hist /= hist_sum
            
        return hist


    def build_vocabulary_and_database(self):
        if not self.training_descriptors:
            self.get_logger().warning("No features gathered! Cannot build vocabulary.")
            return False

        self.get_logger().info("Clustering visual words... This may take a moment.")
        
        # Train the vocabulary using all gathered descriptors
        bow_train = cv2.BOWKMeansTrainer(self.vocabulary_size)
        for desc in self.training_descriptors:
            bow_train.add(np.float32(desc))
            
        self.vocabulary = bow_train.cluster()
        
        # Build the database histograms manually
        self.database_histograms = []
        for desc in self.training_descriptors:
            hist = self._compute_histogram(desc)
            self.database_histograms.append(hist)

        self.is_trained = True
        self.get_logger().info("Vocabulary and Database successfully built!")
        return True


    def save_cb(self, request, response):
        path = request.data
        if not path or not os.path.exists(path) or not os.path.isdir(path):
            response.success = False
            response.message = "Path cannot be empty or invalid"
            return response

        if not self.is_trained:
            if not self.build_vocabulary_and_database():
                response.success = False
                response.message = "Failed to build vocabulary. Cannot save database."
                rclpy.error(self.get_logger(), response.message)
                return response

        try:
            if self.vocabulary is None:
                raise RuntimeError("No vocabulary built to save. Switch to test mode first.")
                
            np.save(os.path.join(path, "bow_vocabulary.npy"), self.vocabulary)
            np.save(os.path.join(path, "bow_database.npy"), np.array(self.database_histograms))

            with open(os.path.join(path, "bow_poses.txt"), 'w') as f:
                for pose in self.stored_poses:
                    t = pose.transform.translation
                    r = pose.transform.rotation
                    f.write(f"{t.x} {t.y} {t.z} {r.x} {r.y} {r.z} {r.w}\n")

            response.success = True
            response.message = f"Database saved to {path}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to save database: {str(e)}"
            
        return response


    def load_cb(self, request, response):
        path = request.data
        if not path or not os.path.exists(path) or not os.path.isdir(path):
            response.success = False
            response.message = "Path cannot be empty or invalid"
            return response

        try:
            self.vocabulary = np.load(os.path.join(path, "bow_vocabulary.npy"))
            db_hists = np.load(os.path.join(path, "bow_database.npy"))
            self.database_histograms = list(db_hists)

            poses_file = os.path.join(path, "bow_poses.txt")
            self.stored_poses.clear()
            with open(poses_file, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) == 7:
                        pose = TransformStamped()
                        pose.header.frame_id = "map"
                        pose.child_frame_id = "base_footprint"
                        pose.transform.translation.x = float(parts[0])
                        pose.transform.translation.y = float(parts[1])
                        pose.transform.translation.z = float(parts[2])
                        pose.transform.rotation.x = float(parts[3])
                        pose.transform.rotation.y = float(parts[4])
                        pose.transform.rotation.z = float(parts[5])
                        pose.transform.rotation.w = float(parts[6])
                        self.stored_poses.append(pose)

            self.is_trained = True
            self.is_training = False
            response.success = True
            response.message = f"Database loaded from {path}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to load database: {str(e)}"
            rclpy.error(self.get_logger(), response.message)
            
        return response


    def image_callback(self, image_msg, info_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="mono8")
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge exception: {e}")
            return

        keypoints, descriptors = self.detector.detectAndCompute(img, None)

        if descriptors is None or len(descriptors) == 0:
            return

        if self.is_training:
            now = self.get_clock().now()
            time_diff = (now - self.last_train_time).nanoseconds / 1e9
            
            if time_diff < (1.0 / self.training_frequency):
                return
            
            self.last_train_time = now

            try:
                current_pose = self.tf_buffer.lookup_transform(
                    "map", "base_footprint", rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().warning(
                    f"Could not transform: {ex}", throttle_duration_sec=2.0)
                return

            self.training_descriptors.append(descriptors)
            self.stored_poses.append(current_pose)
            self.get_logger().info(
                f"Added image to training set. Total images: {len(self.training_descriptors)}")
                
        else:
            if not self.is_trained:
                if not self.build_vocabulary_and_database():
                    return

            # Compute histogram using our custom matcher logic
            query_hist = self._compute_histogram(descriptors)
            
            if query_hist is not None and self.database_histograms:
                best_score = float('inf')
                best_id = -1
                
                for i, db_hist in enumerate(self.database_histograms):
                    score = cv2.norm(query_hist, db_hist, cv2.NORM_L1)
                    if score < best_score:
                        best_score = score
                        best_id = i

                if best_id != -1 and best_score < 1.5:  
                    pose = self.stored_poses[best_id]
                    pose.child_frame_id = "base_footprint_bow"
                    pose.header.stamp = self.get_clock().now().to_msg()
                    self.transform_broadcaster.sendTransform(pose)

                    self.get_logger().info(
                        f"Relocalized! Match Score: {best_score:.3f} | "
                        f"Pose [x: {pose.transform.translation.x:.2f}, y: {pose.transform.translation.y:.2f}]"
                    )


def main(args=None):
    rclpy.init(args=args)
    node = BagOfWords("bag_of_words")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()