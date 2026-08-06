#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped, PoseStamped
from scipy.spatial.transform import Rotation 
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import tf2_geometry_msgs
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool

# import the chosen callback group type from rclpy.executors
from rclpy.executors import MultiThreadedExecutor

from math import atan2, sin, cos, sqrt

class Controller(Node):
    def __init__(self):
        super().__init__("controller") # Initialize node, name it 'controller'
        self.cb_group = ReentrantCallbackGroup()
        self.sub_odom = self.create_subscription(Odometry , '/odom',self.update_odometry ,10,callback_group=self.cb_group)
        # Subscribe to the '/nav_path' topic that contains Path messages published by the path_generator node
        self.nav_path = self.create_subscription(Path, '/nav_path', self.handle_path, 10, callback_group=self.cb_group) 
       
       # Subscribe to the '/move_base_simple/goal' topic that contains PoseStamped messages published by RVIZ 
        self.goal = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.go_to, 10, callback_group=self.cb_group)

        # Publish TwistStamped messages to the '/cmd_vel' topic.
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.active_path_publisher = self.create_publisher(Bool, '/active_path', 10)
        
        # create a callback group for the '/odom', '/move_base_simple/goal '/nav_path' subscriptions
            # visit ths page of the ROS 2 docs for guidance:
            # https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html#callback-groups
        self.px = 0.0
        self.py = 0.0
        self.pth = 0.0
        self.odom_received = False 

        # create a transform listener + TF buffer
            # https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        # This lets you use the command 
        # MSG_IN_NEW_FRAME = self._tf_buffer.transform(MSG_TO_BE_TRANSFORMED,NEW_FRAME, rclpy.duration.Duration(seconds=1))
        # that I found helpful
        self.fwd_effort = 0.0
        self.ang_effort = 0.0

        self.kp_fwd = 1.25
        self.kp_ang = 1.75
     #   pass # delete this before you run your code


    def update_odometry(self, msg: Odometry):
        '''
        A callback that updates the current pose of the robot
        :param msg [Odometry] the current odometry information
        '''
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        
        quat = msg.pose.pose.orientation
        
        self.pth = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')[2]

        self.odom_received = True
       # pass  # delete this before you run your code
    

    def send_speed(self, linear_speed: float, angular_speed: float): 
        '''
        Sends speed to the /cmd_vel topic, which runs the turtlebot motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        '''

        # TODO: publish the message, 
        # hint: remember what message type the topic '/cmd_vel' expects
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear_speed
        msg.twist.angular.z = angular_speed

        self.publisher_.publish(msg)
       # pass  # delete this before you run your code


    def drive(self, distance: float, linear_speed: float):
        """
        Drive straight while continuously refreshing the physical
        TurtleBot's velocity command.
        """
        distance_tolerance = 0.105

        start_x = self.px
        start_y = self.py

        rate = self.create_rate(10.0)
        try:
            while rclpy.ok():
                distance_covered = sqrt((self.px - start_x)**2 +(self.py-start_y)**2)

                if distance_covered >= distance - distance_tolerance:
                    break

                distance_error = distance - distance_covered
                self._logger.info(f"Distance error: {distance_error:.2f} m")
                
                self.send_speed(distance_error * self.kp_fwd, 0.0)
                rate.sleep()

            # self.send_speed(0.0, 0.0)
        except Exception as e:
            self.get_logger().error(f"Error driving: {e}")
            self.send_speed(0.0, 0.0)

    def rotate(self, angle: float, angular_speed: float):
        '''
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        '''
        angle_tolerance = 0.17

        target_theta = atan2(sin(self.pth + angle),cos(self.pth + angle))

        rate = self.create_rate(10.0)

        try:
            while rclpy.ok():
                
                angle_error = atan2(sin(target_theta - self.pth),cos(target_theta - self.pth))

                if abs(angle_error) <= angle_tolerance:
                    break
                    
                direction = 1.0 if angle_error > 0.0 else -1.0

                self.ang_effort = direction * abs(angular_speed) * self.kp_ang * abs(angle_error)

                self.send_speed(0.0,self.ang_effort)
                rate.sleep()
                self._logger.info(f"Angle error: {angle_error:.2f} rad")
            self.send_speed(0.0, 0.0)
        except Exception as e:
            self.get_logger().error(f"Error rotating: {e}")
            self.send_speed(0.0, 0.0)

      #  pass  # delete this before you run your code


    def go_to(self, msg: PoseStamped):
        '''
        Uses rotate() and drive() to get to a specific pose.
        :param msg [PoseStamped] The target or "goal" pose.
        '''

        if not hasattr(self, 'odom_received') or not self.odom_received:
            self.get_logger().warn("Waiting for initial odometry message...")
            return

        msg.header.stamp = self.get_clock().now().to_msg()
        goal = self._tf_buffer.transform(msg,'odom',timeout=rclpy.duration.Duration(seconds=1.0))
        dx = goal.pose.position.x - self.px
        dy = goal.pose.position.y - self.py
        target_angle = atan2(dy, dx)
        angle_error = atan2(sin(target_angle - self.pth),cos(target_angle - self.pth))
        distance = sqrt(dx ** 2 + dy ** 2)
        self.rotate(angle_error, 0.5)
        self.drive(distance, 0.2)

        self.send_speed(0.0, 0.0)

        #quat = goal.pose.orientation

        #goal_yaw = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')[2]

        #final_angle_error = atan2(sin(goal_yaw - self.pth),cos(goal_yaw - self.pth))

        #self.rotate(final_angle_error, 0.5)

       # pass  # delete this before you run your code
     
    
    def handle_path(self, path:Path):
        '''
        A callback function that handles iterating through a path.
        :param path [Path] The path that the robot will drive.
        '''
        if not path.poses:
            self.get_logger().warning('Received an empty path')
            self.send_speed(0.0, 0.0)
            return

        self.active_path_publisher.publish(Bool(data=True))

        # for i in range(len(path.poses)): # n = #poses to skip 
        #     n = 2
        #     if i >= (len(path.poses)-n) : self.go_to(path.poses[i])
        
        #     else: self.go_to(path.poses[i + n])
        self.get_logger().info(f"Received path with {len(path.poses)} poses. Starting to follow the path.")
        step = 0
        for pose in path.poses[1::]:
            self._logger.info(f"Following pose {step + 1}/{len(path.poses) - 1}")
            self.go_to(pose)
            step += 1
        self.send_speed(0.0, 0.0)
        self.active_path_publisher.publish(Bool(data=False))

    def smooth_drive(self, distance: float, linear_speed: float):
        '''
        Smoothly drives the robot in a straight line by regulating its speed.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        '''
        # EXTRA CREDIT

        #pass  # delete this before you run your code

    
def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        try:
            node.send_speed(0.0, 0.0)
        except Exception as e:
            node.get_logger().error(f"Error stopping robot: {e}")
    finally:
        executor.shutdown()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
if __name__ == '__main__':
    main()
