#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

import tf2_geometry_msgs

class DummyClient(Node):

    def __init__(self):
        super().__init__('Dummy_Path_Planner_Client')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.cb_group = ReentrantCallbackGroup() # allow for multiple things at once


        self.cli = self.create_client(GetPlan, 'request_plan',callback_group=self.cb_group)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')


        self.req = GetPlan.Request()


        self.publisher_ = self.create_publisher(Path, '/nav_path', 10)

        self.subscription = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.send_request,
            10,
            callback_group=self.cb_group)
    
    def send_request(self, goal):

        #get robots pose in map frame
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'base_link'
        start_pose.header.stamp = goal.header.stamp
        #self.get_logger.info("This is send_request")
        self.req.start = start_pose
        self.req.goal = goal

        self.get_logger().info("starting service")

        future = self.cli.call_async(self.req)

        future.add_done_callback(self.publish)  # ← non-blocking
        self.get_logger().info("ending callback")

    def publish(self, future):
        self.publisher_.publish(future.result().plan)





def main(args=None):
    rclpy.init(args=args)

    node = DummyClient()
    executor = MultiThreadedExecutor()

    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
