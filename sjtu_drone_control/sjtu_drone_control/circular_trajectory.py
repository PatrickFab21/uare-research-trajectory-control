#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class CircularTrajectoryNode(Node):
    def __init__(self):
        super().__init__('circular_trajectory_node')

        # Publisher to send velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_trajectory)

        # Define trajectory parameters
        self.linear_speed = 30.0 #Forward speed (m/s)
        self.circular_radius = 53.13 #Radius of the circular trajectory (m)
        self.angular_speed = self.linear_speed / self.circular_radius #Angular velocity (rad/s)
    
    def publish_trajectory(self):
        """
        Publishes a Twist message to follow a circular trajectory
        """
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed  # Forward movement
        twist_msg.angular.z = self.angular_speed  # Rotational movement

        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info(f'Circular trajectory: Linear={self.linear_speed}, Angular={self.angular_speed}')

def main(args=None):
    rclpy.init(args=args)
    node = CircularTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
