#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
import math
from tf_transformations import quaternion_matrix, quaternion_from_matrix, translation_from_matrix, inverse_matrix, concatenate_matrices


class PDMotionPlanner(Node):
    def __init__(self):
        super().__init__("pd_motion_planner_node")

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parameters
        self.declare_parameter("kp", 2.0)
        self.declare_parameter("kd", 0.1)
        self.declare_parameter("step_size", 0.2)
        self.declare_parameter("max_linear_velocity", 0.3)
        self.declare_parameter("max_angular_velocity", 1.0)

        self.kp = self.get_parameter("kp").value
        self.kd = self.get_parameter("kd").value
        self.step_size = self.get_parameter("step_size").value
        self.max_linear_velocity = self.get_parameter("max_linear_velocity").value
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").value

        # Subscribers and publishers
        self.path_sub = self.create_subscription(Path, "/a_star/path", self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.next_pose_pub = self.create_publisher(PoseStamped, "/pd/next_pose", 10)

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        self.global_plan = None

        self.prev_angular_error = 0.0
        self.prev_linear_error = 0.0
        self.last_cycle_time = self.get_clock().now()

    def path_callback(self, path: Path):
        self.global_plan = path

    def control_loop(self):
        if not self.global_plan or not self.global_plan.poses:
            return

        # Get the robot's current pose in the odom frame
        try:
            robot_pose_transform = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", rclpy.time.Time())
        except Exception as ex:
            self.get_logger().warn(f"Could not transform: {ex}")
            return

        # Transform plan to robot's frame
        if not self.transform_plan(robot_pose_transform.header.frame_id):
            self.get_logger().error("Unable to transform Plan in robot's frame")
            return

        robot_pose = PoseStamped()
        robot_pose.header.frame_id = robot_pose_transform.header.frame_id
        robot_pose.pose.position.x = robot_pose_transform.transform.translation.x
        robot_pose.pose.position.y = robot_pose_transform.transform.translation.y
        robot_pose.pose.orientation = robot_pose_transform.transform.rotation

        next_pose: PoseStamped = self.get_next_pose(robot_pose)
        dx = next_pose.pose.position.x - robot_pose.pose.position.x
        dy = next_pose.pose.position.y - robot_pose.pose.position.y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        if distance <= 0.1:
            self.get_logger().info("Goal Reached!")
            self.global_plan.poses.clear()
            return

        self.next_pose_pub.publish(next_pose)

        # Calculate the PDMotionPlanner command
        # Transform robot pose and next pose into matrices
        robot_tf = quaternion_matrix([
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z,
            robot_pose.pose.orientation.w,
        ])
        robot_tf[0][3] = robot_pose.pose.position.x
        robot_tf[1][3] = robot_pose.pose.position.y

        next_pose_tf = quaternion_matrix([
            next_pose.pose.orientation.x,
            next_pose.pose.orientation.y,
            next_pose.pose.orientation.z,
            next_pose.pose.orientation.w,
        ])
        next_pose_tf[0][3] = next_pose.pose.position.x
        next_pose_tf[1][3] = next_pose.pose.position.y

        # Compute relative transform: next_pose_robot_tf = robot_tf.inverse() * next_pose_tf
        next_pose_robot_tf = concatenate_matrices(inverse_matrix(robot_tf), next_pose_tf)

        # Extract relative position and orientation
        angular_error = next_pose_robot_tf[1, 3]
        linear_error = next_pose_robot_tf[0, 3] 

        dt = (self.get_clock().now() - self.last_cycle_time).nanoseconds * 1e-9

        angular_error_derivative = (angular_error - self.prev_angular_error) / dt
        linear_error_derivative = (linear_error - self.prev_linear_error) / dt

        cmd_vel = Twist()
        cmd_vel.angular.z = max(
            -self.max_angular_velocity,
            min(self.kp * angular_error + self.kd * angular_error_derivative, self.max_angular_velocity)
        )
        cmd_vel.linear.x = max(
            -self.max_linear_velocity,
            min(self.kp * linear_error + self.kd * linear_error_derivative, self.max_linear_velocity)
        )

        self.cmd_pub.publish(cmd_vel)
        self.prev_angular_error = angular_error
        self.prev_linear_error = linear_error
        self.last_cycle_time = self.get_clock().now()

    def get_next_pose(self, robot_pose: PoseStamped) -> PoseStamped:
        next_pose = self.global_plan.poses[-1]
        for pose in reversed(self.global_plan.poses):
            dx = pose.pose.position.x - robot_pose.pose.position.x
            dy = pose.pose.position.y - robot_pose.pose.position.y
            distance = math.sqrt(dx * dx + dy * dy)
            if distance > self.step_size:
                next_pose = pose
            else:
                break
        return next_pose

    def transform_plan(self, frame):
        if self.global_plan.header.frame_id == frame:
            return True

        try:
            transform = self.tf_buffer.lookup_transform(
                frame, self.global_plan.header.frame_id, rclpy.time.Time())
        except Exception as ex:
            self.get_logger().error(
                f"Couldn't transform plan from frame {self.global_plan.header.frame_id} to {frame}: {ex}")
            return False
        
        transform_matrix = quaternion_matrix([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ])
        transform_matrix[0][3] = transform.transform.translation.x
        transform_matrix[1][3] = transform.transform.translation.y

        for pose in self.global_plan.poses:
            pose_matrix = quaternion_matrix([
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ])
            pose_matrix[0][3] = pose.pose.position.x
            pose_matrix[1][3] = pose.pose.position.y

            transformed_pose = concatenate_matrices(pose_matrix, transform_matrix)
            [pose.pose.orientation.x,pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w] = quaternion_from_matrix(transformed_pose)
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z] = translation_from_matrix(transformed_pose)

        self.global_plan.header.frame_id = frame
        return True


def main(args=None):
    rclpy.init(args=args)
    pd_motion_planner = PDMotionPlanner()
    rclpy.spin(pd_motion_planner)
    pd_motion_planner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
