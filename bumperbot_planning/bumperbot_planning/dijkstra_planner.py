#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException
from queue import PriorityQueue

class GraphNode:
    def __init__(self, x, y, cost=0, prev=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.prev = prev
    
    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __add__(self, other):
        return GraphNode(self.x + other[0], self.y + other[1])

class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__("dijkstra_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/costmap", self.map_callback, map_qos
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 10
        )
        self.path_pub = self.create_publisher(Path, "/dijkstra/path", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/dijkstra/visited_map", 10)

        self.map_ = None
        self.visited_map_ = OccupancyGrid()

    def map_callback(self, map_msg: OccupancyGrid):
        self.map_ = map_msg
        self.visited_map_.header.frame_id = map_msg.header.frame_id
        self.visited_map_.info = map_msg.info
        self.visited_map_.data = [-1] * (map_msg.info.height * map_msg.info.width)

    def goal_callback(self, pose: PoseStamped):
        if self.map_ is None:
            self.get_logger().error("No map received!")
            return

        self.visited_map_.data = [-1] * (self.visited_map_.info.height * self.visited_map_.info.width)

        try:
            map_to_base_tf = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, "base_footprint", rclpy.time.Time()
            )
        except LookupException:
            self.get_logger().error("Could not transform from map to base_footprint")
            return

        map_to_base_pose = Pose()
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x
        map_to_base_pose.position.y = map_to_base_tf.transform.translation.y
        map_to_base_pose.orientation = map_to_base_tf.transform.rotation

        path = self.plan(map_to_base_pose, pose.pose)
        if path.poses:
            self.get_logger().info("Shortest path found!")
            self.path_pub.publish(path)
        else:
            self.get_logger().warn("No path found to the goal.")

    def plan(self, start: Pose, goal: Pose):
        # Define possible movement directions
        explore_directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        # Priority queue with custom comparison for A* based on cost + heuristic
        pending_nodes = PriorityQueue()
        visited_nodes = set()

        start_node = self.world_to_grid(start)
        pending_nodes.put(start_node)

        while not pending_nodes.empty() and rclpy.ok():
            active_node = pending_nodes.get()

            # Goal found!
            if active_node == self.world_to_grid(goal):
                break
            
            # Explore neighbors
            for dir_x, dir_y in explore_directions:
                new_node: GraphNode = active_node + (dir_x, dir_y)
                
                if (new_node not in visited_nodes and self.pose_on_map(new_node) and 
                    0 <= self.map_.data[self.pose_to_cell(new_node)] < 99):
                    
                    new_node.cost = active_node.cost + 1 + self.map_.data[self.pose_to_cell(new_node)]
                    new_node.prev = active_node

                    pending_nodes.put(new_node)
                    visited_nodes.add(new_node)

            self.visited_map_.data[self.pose_to_cell(active_node)] = 10 # Blue
            self.map_pub.publish(self.visited_map_)

        path = Path()
        path.header.frame_id = self.map_.header.frame_id
        while active_node and active_node.prev and rclpy.ok():
            last_pose: Pose = self.grid_to_world(active_node)
            last_pose_stamped = PoseStamped()
            last_pose_stamped.header.frame_id = self.map_.header.frame_id
            last_pose_stamped.pose = last_pose
            path.poses.append(last_pose_stamped)
            active_node = active_node.prev

        path.poses.reverse()
        return path

    def pose_on_map(self, node: GraphNode):
        return 0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.height

    def world_to_grid(self, pose: Pose) -> GraphNode:
        grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        return GraphNode(grid_x, grid_y)

    def grid_to_world(self, node: GraphNode) -> Pose:
        pose = Pose()
        pose.position.x = node.x * self.map_.info.resolution + self.map_.info.origin.position.x
        pose.position.y = node.y * self.map_.info.resolution + self.map_.info.origin.position.y
        return pose

    def pose_to_cell(self, node: GraphNode):
        return node.y * self.map_.info.width + node.x


def main(args=None):
    rclpy.init(args=args)
    node = DijkstraPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
