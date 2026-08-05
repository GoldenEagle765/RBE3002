#!/usr/bin/env python3
from __future__ import annotations
from typing import List, Tuple
import numpy.typing as npt

from rclpy.node import Node
import time
import numpy as np
import math
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Point, PoseStamped
from  nav_msgs.msg import OccupancyGrid, Path, MapMetaData, GridCells
from nav_msgs.srv import GetPlan
import cv2 as cv
from rclpy.executors import MultiThreadedExecutor
import math

from geometry_msgs.msg import Point, PointStamped, PoseStamped
from priority_queue import PriorityQueue 

import tf2_geometry_msgs
from visualization_msgs.msg import Marker

class GraphNode():
    def __init__(self,pose,parent,cost):
        self.pose = pose
        self.parent = parent
        self.cost = cost
    
    def __eq__(self,other):
        return self.pose[0] == other.pose[0] and \
               self.pose[1] == other.pose[1]
    
    def __lt__(self,other):

        return id(self) < id(other)


class PathPlanner(Node):
    def __init__(self):
        # INDIVIDUAL 

        super().__init__("path_planner") # Initialize the node and call it "path_planner"
        self.map_frame = 'map'
        self.declare_parameter('padding', 3.5)
        self.declare_parameter('safe_threshold', 30)
        self.declare_parameter('obstacle_threshold', 60)
        self.cb_group = ReentrantCallbackGroup()

        # Create Quality of Service (QoS) policy. Include a profile, depth, and durablilty policy. 
        qos_profile = QoSProfile(depth = 1,durability = QoSDurabilityPolicy.TRANSIENT_LOCAL, history=QoSHistoryPolicy.KEEP_LAST)

        self.sub_map = self.create_subscription(OccupancyGrid, '/map', 
        self.handle_map, qos_profile, callback_group=self.cb_group) # Subscribe to the map topic

        self.sub_point = self.create_subscription(PointStamped, '/clicked_point', self.handle_click, 10, callback_group=self.cb_group) # Subscribe to the clicked_point topic

        self.pub_safe = self.create_publisher(OccupancyGrid, '/map/safe', qos_profile) # Create a new publisher for '/map/safe' that publishes a message of type OccupancyGrid


        # TODO: Create other publishers with varying message types (GridCells, etc) for visualizing data in Rviz
        self.visited_publisher = self.create_publisher(GridCells, '/path_planner/visited', 10)

        self.centroid_publisher = self.create_publisher(Marker, '/goal_point', 10)



        # TODO: Create service to process GetPlan service requests. See https://docs.ros.org/en/humble/p/nav_msgs/srv/GetPlan.html
        
        self.plan_service = self.create_service(GetPlan,'request_plan',self.plan_path,callback_group=self.cb_group)
        

        self.get_logger().info("path planner node initalized!")

        # numpy array of cell values
        self.map = np.zeros((0,0))

        # Setup TF Buffer and listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.safe_threshold = int(self.get_parameter('safe_threshold').value)
        self.obstacle_threshold = int(self.get_parameter('obstacle_threshold').value)

    def handle_click(self, msg: PointStamped):
        if self.map.size == 0:
            self.get_logger().warning("No map has been received yet.")
            return

        cell = self.world_to_grid(self.map_info, msg.point)
        x, y = cell

        if 0 <= x < self.map_info.width and 0 <= y < self.map_info.height:
            value = int(self.map[y, x])
            state = "safe" if value <= self.safe_threshold and value >= 0 else "unsafe"
            self.get_logger().info(
                f"Clicked point is in cell {cell}: value={value} ({state})"
            )
        else:
            self.get_logger().warning(
                f"Clicked point maps to cell {cell}, which is outside the map."
            )

    @staticmethod
    def obstacle_expansion( original_map:npt.NDArray[np.int32], padding:int, safety_threshold:int=25, obstacle_threshold:int=70) -> npt.NDArray[np.int32]:
        """
        Expands obstacles to be "padding" larger in all directions
        :param original_map     ndarray     Map of obstacles
        :param padding          int         Number of cells from obstacle to consider unsafe
        :param safety_threshold int      Value above which a cell is considered safe
        :return                 ndarray     Map of safe configuration space.
        """
        padding = max(0, int(padding))
        safety_threshold = max(0, min(100, int(safety_threshold)))
        obstacle_threshold = max(0, min(100, int(obstacle_threshold)))

        # Get the unknown cells and occupied cells from the original map (boolean masks)
        
        unknown_mask_unexplored = (original_map < 0).astype(bool)
        unknown_mask_uncertain = ((original_map > safety_threshold) & (original_map < obstacle_threshold)).astype(bool)

        unknown_mask = np.logical_or(unknown_mask_unexplored, unknown_mask_uncertain)

        occupied_mask = (original_map >= obstacle_threshold).astype(bool)

        # bool -> uint8 so CV can understand data type
        obstacle_mask = occupied_mask.astype(np.uint8)

        if padding > 0:        
            kernel_size = 2 * padding + 1
            kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
            obstacle_mask = cv.dilate(obstacle_mask, kernel, iterations=1)

        # uint8 -> boolean mask
        obstacle_mask = obstacle_mask.astype(bool)

        # Create new map with same dimesions apply the unknown and obstacle masks to the new map
        safe_map = np.zeros(original_map.shape, dtype=np.int32)
        safe_map[unknown_mask] = -1
        safe_map[obstacle_mask] = 100
        return safe_map



    def handle_map(self, map:OccupancyGrid):
        """
        Recieve raw map, convert to numpy array, expand obstacles.
        Save safe map and republish. 
        :param map      OccupancyGrid   The current map.
        """
        # INDIVIDUAL
        self.get_logger().info("map received")

        # TODO Convert map to 2D numpy array (See np.asarray)
        width = int(map.info.width)
        height = int(map.info.height)
        original_map = np.asarray(map.data, dtype=np.int32).reshape((height, width))
        
        # TODO Expand obstacles so your map represents where the robots center could be. 

        padding = int(self.get_parameter('padding').value)
        safety_threshold = int(self.get_parameter('safe_threshold').value)
        padded_map = self.obstacle_expansion(original_map, padding, safety_threshold)
        # TODO Store safe numpy array map to self.map and MapMetaData as member variables
        self.map = padded_map
        self.map_info = map.info
        self.map_frame = map.header.frame_id if map.header.frame_id else 'map'

        # TODO Create OccupancyGrid from safe numpy array map
        safe_message = OccupancyGrid()
        safe_message.header = map.header
        safe_message.info = map.info
        safe_message.data = padded_map.reshape(-1).tolist()

        # TODO Publish safe OccupancyGrid
        self.pub_safe.publish(safe_message)
        self.get_logger().info(f"published safe map using padding={padding} cell(s)")
        #  pass


    @staticmethod
    def grid_to_world(mapdata: MapMetaData, p: tuple[int, int]) -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [MapMetaData] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        # INDIVIDUAL
        x, y = p
        resolution = mapdata.resolution

        local_x = (x + 0.5) * resolution
        local_y = (y + 0.5) * resolution

        q = mapdata.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        point = Point()
        point.x = (mapdata.origin.position.x+ math.cos(yaw) * local_x- math.sin(yaw) * local_y)
        point.y = (mapdata.origin.position.y + math.sin(yaw) * local_x + math.cos(yaw) * local_y)
        point.z = mapdata.origin.position.z
        return point
        #pass


        
    @staticmethod
    def world_to_grid(mapdata: MapMetaData, wp: Point) -> tuple[int, int]:
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [MapMetaData] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        # INDIVIDUAL
        dx = wp.x - mapdata.origin.position.x
        dy = wp.y - mapdata.origin.position.y

        q = mapdata.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

        x = math.floor(local_x / mapdata.resolution)
        y = math.floor(local_y / mapdata.resolution)
        return int(x), int(y)
        #pass


    @staticmethod
    def euclidean_distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
        """
        Calculates the Euclidean distance between two points.
        :param p1 [(float, float)] first point.
        :param p2 [(float, float)] second point.
        :return   [float]          distance.
        """
        # INDIVIDUAL
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        #pass
        

    def build_path_message(self, path: list[tuple[int, int]]) -> Path:
        """
        Converts a list of cell coordinates to a Path Message
        :param path     [(int,int)]     The cell coordinates corresponding to the current map
        :return         Path            
        """
        # INDIVIDUAL
        path_message = Path()
        path_message.header.stamp = self.get_clock().now().to_msg()
        path_message.header.frame_id = self.map_frame

        for index, cell in enumerate(path):
            pose = PoseStamped()
            pose.header = path_message.header
            pose.pose.position = self.grid_to_world(self.map_info, cell)

            if len(path) == 1:
                yaw = 0.0
            elif index < len(path) - 1:
                next_cell = path[index + 1]
                yaw = math.atan2(next_cell[1] - cell[1],next_cell[0] - cell[0])
            else:
                previous_cell = path[index - 1]
                yaw = math.atan2(cell[1] - previous_cell[1],cell[0] - previous_cell[0])

            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path_message.poses.append(pose)

        return path_message
        # pass


    def neighbors_of_4(self,p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the safe 4-neighbors cells of (x,y) in the occupancy grid.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable in 4 cardinal directions.
        """
        # INDIVIDUAL
        x, y = p
        neighbors = []

        possible_neighbors = [(x + 1, y),(x - 1, y),(x, y + 1),(x, y - 1)]

        for nx, ny in possible_neighbors:
            if (0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height and self.map[ny, nx] == 0):
                neighbors.append((nx, ny))

        return neighbors
        #pass

    
    def neighbors_of_8(self, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the safe 8-neighbors cells of (x,y) in the occupancy grid.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable in 4 cardinal directions.
        """
        # INDIVIDUAL
        x, y = p
        neighbors = []

        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue

                nx = x + dx
                ny = y + dy

                if not (0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height and self.map[ny, nx] == 0):
                    continue

                if dx != 0 and dy != 0:
                    if self.map[y, nx] != 0 or self.map[ny, x] != 0:
                        continue

                neighbors.append((nx, ny))

        return neighbors
        # pass

    def find_nearest_safe_cell(self, goal_x: int, goal_y: int, search_radius: int = 2) -> Tuple[int, int] | None:
        if 0 <= goal_x < self.map_info.width and 0 <= goal_y < self.map_info.height:
            if self.map[goal_y, goal_x] == 0:
                return (goal_x, goal_y)
        # How many cells to search in every direction around the goal
        for r in range(1, search_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue  # Only check the perimeter of the search box
                    nx, ny = goal_x + dx, goal_y + dy
                    if 0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height:
                        if self.map[ny, nx] == 0:
                            return (nx, ny)
        return None

    def get_edge_cost(self,p1: tuple[int, int],p2: tuple[int, int]):
        """
        Compute cost to traverse between 2 nodes.
        :param p1       [(int, int)]    The coordinate in the grid of start.
        :param p2       [(int, int)]    The coordinate in the grid of end.

        """
        # INDIVIDUAL
        return self.euclidean_distance(p1, p2)
        # pass

    def draw_visited(self,visited):
        """
        Draw set of visited node as GridCells message.
        """
        cells = GridCells()
        cells.cell_width = self.map_info.resolution
        cells.cell_height = self.map_info.resolution
        cells.header.stamp = self.get_clock().now().to_msg()
        cells.header.frame_id = 'map'
        for index in visited:
            p = self.grid_to_world(self.map_info,index[:2])            
            cells.cells.append(p)
        self.visited_publisher.publish(cells)

    def build_path(self,final_node: GraphNode):
        """
        Given goal node from search tree, build path from start to goal to get there
        """

        current_node = final_node
        path = []
        while current_node is not None:
            path.append(current_node.pose)
            current_node= current_node.parent
        return path[::-1]


    def a_star(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Runs A* search from start coordinate to end coordinate using class member map.
        :param start    [[(int,int)]]     start coordinate in map    
        :param goal     [[(int,int)]]      goal coordinate in map    
        :return

        """
        # GROUP

        if self.map.size == 0:
            return []

        for x, y in (start, goal):
            if not (0 <= x < self.map_info.width and 0 <= y < self.map_info.height):
                return []
            if self.map[y, x] != 0:
                return []

        #use GraphNode and PriorityQueue
        frontier = PriorityQueue()
        start_node = GraphNode(start, None, 0.0)
        frontier.put(start_node, self.euclidean_distance(start, goal))

        best_cost = {start: 0.0}
        visited = set()
        
        while not frontier.empty():
            current_node = frontier.get()

            if current_node.pose in visited:
                continue

            visited.add(current_node.pose)

            if current_node.pose == goal:
                self.draw_visited(visited)
                return self.build_path(current_node)

            for neighbor in self.neighbors_of_4(current_node.pose):
                if neighbor in visited:
                    continue

                new_cost = current_node.cost + self.get_edge_cost(current_node.pose,neighbor)

                if new_cost < best_cost.get(neighbor, math.inf):
                    best_cost[neighbor] = new_cost
                    neighbor_node = GraphNode(neighbor,current_node,new_cost)
                    priority = new_cost + self.euclidean_distance(neighbor,goal)
                    frontier.put(neighbor_node, priority)

        self.draw_visited(visited)
        return []

        
        # pass

    def plan_path(self, request, response): 
        """
        Plans a path between the current pose and the goal message locations.
        Internally uses A* to plan the optimal path.
        :param request  nav_msgs.srv._get_plan.GetPlan_Request  Start and End Pose for plan
        :param response nav_msgs.srv._get_plan.GetPlan_Response 
        :return         nav_msgs.srv._get_plan.GetPlan_Response     
        """
        # GROUP
        time.sleep(2)
        # TODO: Add error handling for if there is no map available
        if self.map.size == 0:
            self.get_logger().warning("Cannot plan because no map is available.")
            response.plan = Path()
            response.plan.header.frame_id = self.map_frame
            response.plan.header.stamp = self.get_clock().now().to_msg()
            return response
        # TODO: Find cell index of start and goal poses in map
        #Start Pose to map coordinates
        self._logger.info("Transforming start pose to map frame")
        request.start.header.stamp = rclpy.time.Time().to_msg()
        start_pose = self._tf_buffer.transform(request.start, self.map_frame, timeout=rclpy.duration.Duration(seconds=0.5))
        start = self.world_to_grid(self.map_info, start_pose.pose.position)
        goal = self.world_to_grid(self.map_info, request.goal.pose.position)

        # If goal is unsafe or out of bounds, search for nearest safe cell
        if not (0 <= goal[0] < self.map_info.width and 0 <= goal[1] < self.map_info.height) or self.map[goal[1], goal[0]] != 0:
            self.get_logger().warning(f"Goal {goal} is unsafe or out of bounds. Searching for nearest safe cell...")
            safe_goal = self.find_nearest_safe_cell(goal[0], goal[1], search_radius=3)
            if safe_goal is not None:
                self.get_logger().info(f"Snapped goal from {goal} to safe cell {safe_goal}")
                goal = safe_goal
            else:
                self.get_logger().warning("No safe cells found nearby. Goal is completely unreachable.")
                response.plan = self.build_path_message([])
                return response

        # TODO: Calculate a path using A* 
        self.get_logger().info(f"Planning from {start} to {goal}.")
        self.publish_goal_marker(goal)
        path = self.a_star(start, goal)

        if path:
            self.get_logger().info(
                f"Found a path containing {len(path)} cells."
            )
        else:
            self.get_logger().warning("A* could not find a path.")

        # TODO: Return your path 
        response.plan = self.build_path_message(path)
        return response

        # pass
    def publish_goal_marker(self, goal: tuple[int, int]):
        """
        Publishes a visualization marker for the goal point.
        :param goal [(int, int)] The cell coordinate of the goal.
        """
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Convert grid coordinates to world coordinates
        world_point = self.grid_to_world(self.map_info, goal)
        marker.pose.position = world_point
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.map_info.resolution * 1.25
        marker.scale.y = self.map_info.resolution * 1.25
        marker.scale.z = 1.0

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 1.0

        self.centroid_publisher.publish(marker)

    @staticmethod
    def optimize_path(path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """   
        # EXTRA CREDIT
        pass


    def neighbors_with_orientation(self,p: tuple[int, int,int]) -> list[tuple[int, int, int]]:
        """
        Returns the safe neighbours for nonholonomic robot with pose x,y,theta.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int, int)]    The coordinate in the grid.
        :return        [[(int,int, int)]]   A list of walkable 8-neighbors.
        """
        # EXTRA CREDIT      
        pass



def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
        pass

    except KeyboardInterrupt:
        pass
    finally:
        # TODO: Destroy and shutdown node when ctrl + C is pressed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        pass

if __name__ == '__main__':
    main()
