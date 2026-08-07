#!/usr/bin/env python3

import math
from typing import Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (PointStamped,PoseStamped,PoseWithCovarianceStamped)
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class Localizer(Node):
    def __init__(self) -> None:
        super().__init__("localizer")

        self.declare_parameter("covariance_threshold", 0.125)

        self.covariance_threshold = float(self.get_parameter("covariance_threshold").value)

        
        self.latest_amcl_pose: Optional[PoseWithCovarianceStamped] = None

        self.pending_goal: Optional[PointStamped] = None
        self.localized = False

        self.plan_request_in_progress = False

        amcl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.amcl_pose_subscription = self.create_subscription(PoseWithCovarianceStamped,"/amcl_pose",self.amcl_pose_callback,qos_profile=amcl_qos)

        self.clicked_point_subscription = self.create_subscription(PointStamped,"/clicked_point",self.clicked_point_callback,10)

        self.plan_client = self.create_client(GetPlan,"/request_plan")
        self.path_publisher = self.create_publisher(Path,"/nav_path",10)

        # Retry the request if the planner service was not ready when the
        # covariance or clicked point first arrived.
        self.request_timer = self.create_timer(
            0.5,
            self.try_request_path,
        )

        self.get_logger().info("Localizer started. Waiting for /amcl_pose and /clicked_point.")

    def amcl_pose_callback(self,message: PoseWithCovarianceStamped) -> None:
        self.latest_amcl_pose = message
        if self.localized:
            return

        covariance = message.pose.covariance
        x_variance = float(covariance[0])
        y_variance = float(covariance[7])
        yaw_variance = float(covariance[35])
        mean_covariance = (x_variance + y_variance + yaw_variance) / 3.0

        self.get_logger().info(
            "AMCL covariance: "
            f"x={x_variance:.5f}, "
            f"y={y_variance:.5f}, "
            f"yaw={yaw_variance:.5f}, "
            f"mean={mean_covariance:.5f}"
        )

        if mean_covariance < self.covariance_threshold:
            self.localized = True

            self.get_logger().info(
                "Localization accepted. "
                f"Mean covariance {mean_covariance:.5f} is below "
                f"{self.covariance_threshold:.5f}."
            )
            self.try_request_path()

    def clicked_point_callback(self,message: PointStamped) -> None:

        if self.plan_request_in_progress:
            self.get_logger().warning(
                "A path request is already in progress. "
                "Ignoring the new clicked point.")
            return

        self.pending_goal = message

        self.get_logger().info(
            "Received clicked point: "
            f"x={message.point.x:.3f}, "
            f"y={message.point.y:.3f}, "
            f"frame={message.header.frame_id}"
        )

        if not self.localized:
            self.get_logger().info(
                "Waiting for AMCL covariance to fall below "
                f"{self.covariance_threshold:.3f}."
            )
            return

        self.try_request_path()

    def try_request_path(self) -> None:
    

        if not self.localized:
            return

        if self.latest_amcl_pose is None:
            return

        if self.pending_goal is None:
            return

        if self.plan_request_in_progress:
            return

        if not self.plan_client.service_is_ready():
            self.get_logger().info(
                "Waiting for the /request_plan service."
            )
            return

        goal_frame = self.pending_goal.header.frame_id

        if not goal_frame:
            goal_frame = "map"


        request = GetPlan.Request()

        request.start = PoseStamped()

        request.start.header = self.latest_amcl_pose.header

        if not request.start.header.frame_id:
            request.start.header.frame_id = "map"

        request.start.pose = self.latest_amcl_pose.pose.pose
        request.goal = PoseStamped()

        request.goal.header = self.pending_goal.header
        request.goal.header.frame_id = goal_frame

        request.goal.pose.position = self.pending_goal.point

        request.goal.pose.orientation.x = 0.0
        request.goal.pose.orientation.y = 0.0
        request.goal.pose.orientation.z = 0.0
        request.goal.pose.orientation.w = 1.0

        request.tolerance = 0.0

        self.get_logger().info(
            "Requesting path from "
            f"({request.start.pose.position.x:.3f}, "
            f"{request.start.pose.position.y:.3f}) "
            "to "
            f"({request.goal.pose.position.x:.3f}, "
            f"{request.goal.pose.position.y:.3f})."
        )

        self.plan_request_in_progress = True

        future = self.plan_client.call_async(request)
        future.add_done_callback(self.plan_response_callback)

    def plan_response_callback(self, future) -> None:

        self.plan_request_in_progress = False

        try:
            response = future.result()

        except Exception as exception:
            self.get_logger().error(
                f"Path-planning service failed: {exception}"
            )
            return

        if response is None:
            self.get_logger().error(
                "The path planner returned no response."
            )
            return

        path = response.plan

        if not path.poses:
            self.get_logger().warning(
                "The path planner returned an empty path."
            )
            return

        self.get_logger().info(
            f"Publishing path with {len(path.poses)} poses to /nav_path."
        )

        self.path_publisher.publish(path)
        self.pending_goal = None


def main(args=None) -> None:
    rclpy.init(args=args)

    node = Localizer()
    

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()