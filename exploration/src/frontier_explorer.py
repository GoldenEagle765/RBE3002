#!/usr/bin/env python3

from __future__ import annotations
from typing import List, Tuple
import numpy as np
import rclpy
import numpy.typing as npt
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from  nav_msgs.msg import OccupancyGrid, Path, MapMetaData, GridCells
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
import cv2 as cv
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.srv import GetPlan
import yaml
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")


        self.cb_group = ReentrantCallbackGroup()
        qos_profile = QoSProfile(depth = 1,durability = QoSDurabilityPolicy.TRANSIENT_LOCAL, history=QoSHistoryPolicy.KEEP_LAST)

        # Sub to safe map
        self.sub_safe_map = self.create_subscription(OccupancyGrid, '/map/safe', self.handle_safe_map, qos_profile, callback_group=self.cb_group)

        self.cli = self.create_client(GetPlan, 'request_plan',callback_group=self.cb_group)
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = GetPlan.Request()

        self.publisher_ = self.create_publisher(Path, '/nav_path', 10)

        self.path_subscriber = self.create_subscription(Bool, '/active_path', self.handle_path_state, 10, callback_group=self.cb_group)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.frontier_publisher = self.create_publisher(GridCells, '/frontiers', 10)
        self.centroid_publisher = self.create_publisher(Marker, '/frontier_centroids', 10)

        self.has_path = False

    # def curr_cell(self, cell: Point):
    #     curr_cell = cell
    #     curr_cell.x = PathPlanner.world_to_grid.x 
    #     curr_cell.y = PathPlanner.world_to_grid.y
    #     return self.curr_cell

    # Save safe map, find frontiers, choose explore point
    def handle_path_state(self, msg: Bool):
        if msg.data == False:
            self.has_path = False
            self._logger.info("No active path")
        else:
            self.has_path = True

    def handle_safe_map(self, msg:OccupancyGrid):
        self._logger.info("Recieved Safe Map")
        self.safe_map = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.map_info = msg.info

        if self.has_path:
            self._logger.info("Active path exists, waiting for completion before accepting new path")
            return

        frontiers = self.find_frontiers()
        self.publish_frontier_cells(frontiers)
        centroids = self.find_centroids(frontiers)

        if(len(centroids)) == 0:
            self._logger.info("No valid centroids found")
            return

        # Get the best of the centroids and request the path
        else:
            self.publish_centroids(centroids)
            goal_point = self.next_target(centroids)
        
        if(goal_point == None):
            self._logger.info("Couldn't find a target")

        else:
            self.send_request(goal_point)

    def save_map(self):
        cv.imwrite('safe_map.png', self.safe_map)
        yaml_info = {
                "image": 'safe_map.png',
                "resolution": self.map_info.resolution,
                "origin": [self.map_info.origin.position.x, self.map_info.origin.position.y, 0.0],
            }
        with open('safe_map.yaml', 'w') as f:
                yaml.dump(yaml_info, f)
                    
                if (cv.imread('safe_map.png') is None):
                    self._logger.info("Failed to save safe_map")
                    
                else: self.destroy_node()


    # Find frontiers in the safe map
    def find_frontiers(self) -> List[np.uint8]:
        # Make a safe mask, make an unknown mask, dilate unknown mask to account for neighbors
        # Create map of cells in both free space and unknown neighbors
        safe_mask = (self.safe_map == 0).astype(np.uint8)
        unknown_mask = (self.safe_map == -1)
        unknown_expanded = cv.dilate(unknown_mask.astype(np.uint8), np.ones((3, 3), np.uint8))
        self._logger.info("Calculated frontier mask")
        unfiltered_frontiers = cv.bitwise_and(safe_mask, unknown_expanded)
        # filtered_frontiers = cv.bitwise_and(unfiltered_frontiers, cv.bitwise_not(unknown_mask.astype(np.uint8)))
        


        # return filtered_frontiers
        return unfiltered_frontiers

    def publish_frontier_cells(self, frontiers: np.ndarray):
        grid_cells_msg = GridCells()
        grid_cells_msg.header.frame_id = 'map'
        grid_cells_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Match grid block size to the map resolution
        grid_cells_msg.cell_width = self.map_info.resolution
        grid_cells_msg.cell_height = self.map_info.resolution

        # Find all pixel coordinates where the frontier mask is active
        y_indices, x_indices = np.where(frontiers > 0)

        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution

        cells_list = []
        for x, y in zip(x_indices, y_indices):
            p = Point()
            p.x = float(origin_x + (x + 0.5) * resolution)
            p.y = float(origin_y + (y + 0.5) * resolution)
            p.z = 0.0
            cells_list.append(p)

        grid_cells_msg.cells = cells_list
        self.frontier_publisher.publish(grid_cells_msg)

    def find_centroids(self, frontiers: List[np.uint8]):
        # For each centroid found if connected_pixels > 3 get the centroid and append to list
        num_labels, labels, stats, centroids = cv.connectedComponentsWithStats(frontiers, connectivity=8)
        valid_centroids = []
        for i in range(1, num_labels):
            if(stats[i, cv.CC_STAT_AREA] > 5):
                cx, cy = centroids[i]
                valid_centroids.append((cx, cy))
        self._logger.info("Found " + str(len(valid_centroids)) + " valid centroids")
        if valid_centroids == 0:
            self.find_centroids(self.find_frontiers())
                
        else: return valid_centroids
        return valid_centroids

    def publish_centroids(self, centroids: List[Tuple[float, float]]):
        try:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'centroids'
            marker.id = 0
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.points = []
            for x, y in centroids:
                p = Point()
                p.x = float((x + 0.5) * self.map_info.resolution + self.map_info.origin.position.x)
                p.y = float((y + 0.5) * self.map_info.resolution + self.map_info.origin.position.y)
                p.z = 0.0
                marker.points.append(p)
            self.centroid_publisher.publish(marker)
        except Exception as e:
            self._logger.error(f"Error publishing centroids: {e}")

    # For each centroid calculate given hueristic to choose best frontier
    # Current heuristic : Euclidean Distance from Start
    def next_target(self, centroids:List[(np.uint8, np.uint8)]):
        target = None
        # To save on computation no sqrt() -> best = a^2 + b^2
        best = 1000.0
        for x, y in centroids:
            world_x = (x + 0.5) * self.map_info.resolution + self.map_info.origin.position.x
            world_y = (y + 0.5) * self.map_info.resolution + self.map_info.origin.position.y

            distance = (world_x ** 2) + (world_y ** 2)

            if distance < best:
                self._logger.info("Best Grid Point : (" + str(x) + "," + str(y) + ")" + " World Point : (" + str(world_x) + "," + str(world_y) + ")" + " Distance : " + str(distance))
                best = distance
                world_x = (x + 0.5) * self.map_info.resolution + self.map_info.origin.position.x
                world_y = (y + 0.5) * self.map_info.resolution + self.map_info.origin.position.y
                target = Point(x = float(world_x), y = float(world_y))
        return target

    def send_request(self, goal: Point):
            # Start Pose
            
            start_pose = PoseStamped()
            start_pose.header.frame_id = 'base_link'
            start_pose.header.stamp = self.get_clock().now().to_msg()
            start_pose.pose.orientation.w = 1.0
            self.req.start = start_pose

            # Goal Pose
            self.req.goal.pose.position = goal
            self.req.goal.header.frame_id = 'map'
            self.req.goal.header.stamp = self.get_clock().now().to_msg()
            self.req.goal.pose.orientation.w = 1.0
    
            self.get_logger().info("starting service")
    
            future = self.cli.call_async(self.req)
    
            future.add_done_callback(self.publish)  # ← non-blocking
            self.get_logger().info("ending callback")
    
    def publish(self, future):
        self.publisher_.publish(future.result().plan)

def main(args=None):    
    rclpy.init(args=args)
    node = FrontierExplorer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()