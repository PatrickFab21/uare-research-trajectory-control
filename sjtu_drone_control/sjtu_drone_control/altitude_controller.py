#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

class AltitudeController(Node):
    def __init__(self):
        super().__init__('altitude_controller')

        # Parameter to define the desired altitude in real time in meters
        self.declare_parameter('target_altitude', 10.0)
        self.target_altitude = self.get_parameter('target_altitude').value

        # Publicator for cmd_vel and takeoff
        self.cmd_vel_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/simple_drone/takeoff', 10)

        # Subscriber for odometry to get the current altitude
        self.odom_sub = self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)
    
        # Control variables
        self.current_altitude = 0.0
        self.tolerance = 0.2  # Acceptable error in meters
        self.has_taken_off = False  # Takeoff flag
        # Timer to adjust the altitude every 0.1 seconds
        self.create_timer(0.1, self.control_altitude)

    def odom_callback(self, msg):
        """Receives the current altitude of the drone from the odometry."""
        self.current_altitude = msg.pose.pose.position.z

    def control_altitude(self):
        """Executes takeoff and automatically adjusts the altitude."""
        # If the drone has not taken off yet, send the takeoff command
        if not self.has_taken_off or self.current_altitude < 0.5:
            self.get_logger().info("Sending takeoff command...")
            self.takeoff_pub.publish(Empty())  # Publish the takeoff command
            self.has_taken_off = True
            return  # Wait before adjusting the altitude

        # Get the updated value of the target altitude parameter
        self.target_altitude = self.get_parameter('target_altitude').value

        # Calculate the altitude error
        error = self.target_altitude - self.current_altitude
        cmd = Twist()

        if abs(error) > self.tolerance:
            cmd.linear.z = 0.5 * error  # Proportional control to adjust altitude
        else:
            cmd.linear.z = 0.0  # If the altitude is correct, do not ascend or descend

        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'Current altitude: {self.current_altitude:.2f} m | Target: {self.target_altitude:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    node = AltitudeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()