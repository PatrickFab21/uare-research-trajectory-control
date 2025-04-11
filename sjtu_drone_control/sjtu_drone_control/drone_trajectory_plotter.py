#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class DroneTrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('drone_trajectory_plotter')

        # Subscribe only to Odometry. We no longer need cmd_vel to stop plotting.
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/simple_drone/odom',
            self.odom_callback,
            10
        )


        # Store coordinates
        self.x_data = []
        self.y_data = []

        # Set up the figure
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Drone Trajectory')
        self.ax.grid(True)

    def odom_callback(self, msg):
        """Receive odometry and store the coordinates."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        self.x_data.append(x)
        self.y_data.append(y)

        self.get_logger().info(f'Posición: x={x:.2f}, y={y:.2f}')
        # Update the plot
        self.ax.clear()
        self.ax.plot(self.x_data, self.y_data, '-r', label="Drone Path")
        self.ax.legend()
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        # Update the subtitle with the current Z position
        self.ax.set_title(f'Drone Trajectory\nCurrent Z Position: {z:.2f} m')
        self.ax.grid(True)
        plt.pause(0.1)  # Pause to update the figure

    def run(self):
        """Keep the node running and continuously update the plot."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        # Once we exit (e.g., Ctrl + C), save and show the final plot
        self.get_logger().info("Shutting down: saving final plot...")
        plt.savefig("drone_trajectory.png")
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = DroneTrajectoryPlotter()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
