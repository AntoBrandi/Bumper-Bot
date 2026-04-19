#!/usr/bin/env python3
import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

import message_filters
import point_cloud_transport_py
from point_cloud_transport_py.common import pointCloud2ToString

class StereoVision(Node):
    def __init__(self, name):
        super().__init__(name)
        
        self.bridge = CvBridge()

        self.left_image_sub = message_filters.Subscriber(
            self, Image, "left_camera/image_raw", qos_profile=qos_profile_sensor_data)
        
        self.left_info_sub = message_filters.Subscriber(
            self, CameraInfo, "left_camera/camera_info", qos_profile=qos_profile_sensor_data)
        
        self.right_image_sub = message_filters.Subscriber(
            self, Image, "right_camera/image_raw", qos_profile=qos_profile_sensor_data)
        
        self.right_info_sub = message_filters.Subscriber(
            self, CameraInfo, "right_camera/camera_info", qos_profile=qos_profile_sensor_data)

        self.time_sync = message_filters.ApproximateTimeSynchronizer(
            [self.left_image_sub, self.left_info_sub, self.right_image_sub, self.right_info_sub],
            queue_size=10,
            slop=0.1
        )
        self.time_sync.registerCallback(self.image_callback)

        self.disparity_image_pub = self.create_publisher(
            Image, "disparity", qos_profile_sensor_data)
        
        self.pct = point_cloud_transport_py.PointCloudTransport("point_cloud_transport", "")
        self.point_cloud_pub = self.pct.advertise("point_cloud", 10)

        self.get_logger().info("Stereo Vision Node Initialized")

    def image_callback(self, left_img_msg, left_info_msg, right_img_msg, right_info_msg):
        """
        Callback triggered only when all 4 messages are available and synchronized.
        """
        try:
            left_img = self.bridge.imgmsg_to_cv2(left_img_msg, desired_encoding="mono8")
            right_img = self.bridge.imgmsg_to_cv2(right_img_msg, desired_encoding="mono8")

            sgbm = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=96,
                blockSize=9,
                P1=8 * 9 * 9,
                P2=32 * 9 * 9,
                disp12MaxDiff=1,
                preFilterCap=63,
                uniquenessRatio=10,
                speckleWindowSize=100,
                speckleRange=32,
                mode=cv2.STEREO_SGBM_MODE_SGBM
            )
            disparity_sgbm = sgbm.compute(left_img, right_img)

            disparity = disparity_sgbm.astype(np.float32) / 16.0
            normalized_disparity = cv2.normalize(
                disparity_sgbm, None, alpha=0, beta=255, 
                norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
            )
            disparity_out_msg = self.bridge.cv2_to_imgmsg(normalized_disparity, encoding="mono8")
            disparity_out_msg.header = left_img_msg.header
            self.disparity_image_pub.publish(disparity_out_msg)

            fx = left_info_msg.k[0]
            cx = left_info_msg.k[2]
            fy = left_info_msg.k[4]
            cy = left_info_msg.k[5]
            
            baseline = 0.06 
            mask = (disparity > 10.0) & (disparity < 96.0)

            if not np.any(mask):
                return

            rows, cols = left_img.shape
            u, v = np.meshgrid(np.arange(cols), np.arange(rows))
            
            u_valid = u[mask]
            v_valid = v[mask]
            d_valid = disparity[mask]
            intensity_valid = left_img[mask]

            z = (fx * baseline) / d_valid
            x = (u_valid - cx) * z / fx
            y = (v_valid - cy) * z / fy
            intensity = intensity_valid.astype(np.float32) / 255.0
            points = np.stack([x, y, z, intensity], axis=-1)

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]

            pc_msg = pc2.create_cloud(left_img_msg.header, fields, points)
            self.point_cloud_pub.publish(pointCloud2ToString(pc_msg))

        except Exception as e:
            self.get_logger().error(f"Error in stereo callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StereoVision("stereo_vision")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()