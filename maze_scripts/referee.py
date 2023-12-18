#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import sys
import argparse

class TimeMeasurementNode(Node):
    def __init__(self, position_x, position_y):
        super().__init__('referee_node')
        self.goal_position = Point(x=position_x, y=position_y, z=0.0)
        self.robot_position = None
        self.time_started = None
        self.distance_threshold = 0.25
        self.time = None

        self.distance_publisher = self.create_publisher(
            Float32,
            '/referee/distance_left',
            10
        )

        self.time_publisher = self.create_publisher(
            Float32,
            '/referee/time_elapsed',
            10
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        self.robot_position = msg.pose.pose.position
        self.publish_distance_left()
        # Check if robot left starting position
        if ((self.robot_position.x**2 + self.robot_position.y**2)**0.5 > self.distance_threshold) and self.time_started is None:
            self.get_logger().info('Robot left starting position. Starting time measurement.')
            self.start_measurement()

        if self.time_started is not None:
            self.publish_time_elapsed()
        
        # Check if robot reached goal position
        if ((self.goal_position.x - self.robot_position.x)**2 + (self.goal_position.y - self.robot_position.y)**2)**0.5 < self.distance_threshold:
            self.get_logger().info("Robot reached goal position.")
            self.get_logger().info("Time elapsed: {} seconds".format(self.time))
            self.destroy_node()
            exit()

    def start_measurement(self):
        self.time_started = self.get_clock().now()

    def publish_time_elapsed(self):
        self.time = (self.get_clock().now() - self.time_started).nanoseconds/10e8
        
        self.time_publisher.publish(Float32(data=self.time))

    def publish_distance_left(self):
        distance_left = Float32()
        distance_left.data = ((self.goal_position.x - self.robot_position.x)**2 + (self.goal_position.y - self.robot_position.y)**2)**0.5
        self.distance_publisher.publish(distance_left)

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(
        prog="RefereeNode",
        description="A node that measures the time elapsed between the start of the simulation and the robot reaching a goal position."
    )
    parser.add_argument(
        "-x",
        "--goal_position_x",
        type=float,
        required=True,
        help="The goal position that the robot must reach in x."
    )
    parser.add_argument(
        "-y",
        "--goal_position_y",
        type=float,
        required=True,
        help="The goal position that the robot must reach in y."
    )
    args, _ = parser.parse_known_args()

    node = TimeMeasurementNode(args.goal_position_x, args.goal_position_y)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
