#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

import math

def quaternion_to_euler(qx, qy, qz, qw):
    """
    Converts quaternion to Euler angles (roll, pitch, yaw).
    """
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class GoToWaypointListController(Node):
    def __init__(self):
        super().__init__('go_to_waypoint_list_controller')

        # --- Numeric parameter to choose the shape ---
        self.declare_parameter('num_vertices', 6)  # default is hexagon
        self.num_vertices = self.get_parameter('num_vertices').get_parameter_value().integer_value

        # --- Define waypoints based on the number of vertices ---
        if self.num_vertices == 3:
            self.waypoints = [
                (28.10, 0.31, 10.0),
                (-13.96, 24.59, 10.0),
                (-13.96, -23.97, 10.0)
            ]
        elif self.num_vertices == 4:
            self.waypoints = [
                (28.21, -0.02, 10.0),
                (0.09, 28.10, 10.0),
                (-28.03, -0.02, 10.0),
                (0.09, -28.13, 10.0)
            ]
        elif self.num_vertices == 5:
            self.waypoints = [
                (27.93, 0.32, 10.0),
                (8.55, 26.99, 10.0),
                (-22.81, 16.80, 10.0),
                (-22.81, -16.17, 10.0),
                (8.55, -26.35, 10.0)
            ]
        elif self.num_vertices == 6:
            self.waypoints = [
                (27.13, -0.33, 10.0),
                (13.17, 23.85, 10.0),
                (-14.75, 23.85, 10.0),
                (-28.71, -0.33, 10.0),
                (-14.75, -24.51, 10.0),
                (13.17, -24.51, 10.0)
            ]
        elif self.num_vertices == 7:
            self.waypoints = [
                (27.47, 0.39, 10.0),
                (16.98, 22.18, 10.0),
                (-6.60, 27.56, 10.0),
                (-25.51, 12.48, 10.0),
                (-25.51, -11.71, 10.0),
                (-6.60, -26.79, 10.0),
                (16.98, -21.40, 10.0)
            ]
        else:
            self.get_logger().error("Invalid number of vertices. Use a value between 3 and 7.")
            self.waypoints = []

        # Display the selected shape
        self.get_logger().info(f'Selected shape with {self.num_vertices} vertices: {self.waypoints}')

        # Index of the current waypoint
        self.current_waypoint_idx = 0

        # --- 2) Control gains and tolerances ---
        #    (You could implement a full PID controller here if desired)
        self.k_yaw = 1.0       # Proportional gain for yaw
        self.k_dist = 0.5      # Proportional gain for XY movement
        self.k_z = 0.5         # Proportional gain for altitude
        self.yaw_threshold = 0.1     # rad
        self.dist_threshold = 0.2    # m
        # Maximum velocities
        self.max_vx = 5.0
        self.max_vz = 1.0
        self.max_wz = 1.5

        # --- 3) Create Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/simple_drone/takeoff', 10)
        # If you had a landing topic, you could:
        # self.land_pub = self.create_publisher(Empty, '/simple_drone/land', 10)

        # --- 4) Subscribe to Odometry ---
        self.odom_sub = self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)

        # --- 5) Internal variables ---
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.has_taken_off = False

        # --- 6) Control timer (10 Hz) ---
        self.timer_period = 0.1
        self.control_timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info('Node [go_to_waypoint_list_controller] started.')

    def odom_callback(self, msg: Odometry):
        """
        Reads the current position and orientation of the drone from odometry.
        Stores x, y, z, and yaw in internal variables.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        self.yaw = yaw  # in radians

    def control_loop(self):
        """
        Main control algorithm to navigate through the list of waypoints:
         1) Ensures takeoff (only once).
         2) Takes the current waypoint (based on self.current_waypoint_idx).
         3) Applies P control for yaw, altitude, and forward movement.
         4) If the distance to the waypoint is < dist_threshold, move to the next waypoint.
         5) Loops back to the first waypoint after completing the list (infinite loop).
        """

        # (1) Take off once
        if not self.has_taken_off or self.z < 0.5:
            self.get_logger().info("Sending takeoff command...")
            self.takeoff_pub.publish(Empty())  # Publish the takeoff command
            self.has_taken_off = True
            return  # Wait before adjusting altitude

        # (2) Get the current waypoint
        #     Example: (27.13, -0.33, 25.0)
        target_x, target_y, target_z = self.waypoints[self.current_waypoint_idx]

        # (3) Calculate errors
        dx = target_x - self.x
        dy = target_y - self.y
        dz = target_z - self.z
        dist_xy = math.sqrt(dx*dx + dy*dy)
        dist_3d = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Desired angle in XY
        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - self.yaw
        # Normalize yaw_error to [-pi, pi]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # (4) Calculate control actions
        cmd = Twist()

        ## 4.1 Yaw control
        wz = self.k_yaw * yaw_error
        # Saturate
        wz = max(-self.max_wz, min(self.max_wz, wz))
        cmd.angular.z = wz

        ## 4.2 Altitude control
        vz = self.k_z * dz
        vz = max(-self.max_vz, min(self.max_vz, vz))
        cmd.linear.z = vz

        ## 4.3 Forward movement in local X
        if abs(yaw_error) < self.yaw_threshold:
            vx = self.k_dist * dist_xy
            vx = max(-self.max_vx, min(self.max_vx, vx))
        else:
            vx = 0.0

        cmd.linear.x = vx
        cmd.linear.y = 0.0  # no lateral movement

        # (5) If close to the target, move to the next waypoint
        #     and loop infinitely (use % for cyclic indexing).
        if dist_3d < self.dist_threshold:
            self.get_logger().info(
                f"Waypoint {self.current_waypoint_idx+1} reached! Current position: "
                f"({self.x:.2f}, {self.y:.2f}, {self.z:.2f}). Moving to the next..."
            )
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)
        else:
            # Or publish info on each iteration:
            self.get_logger().info(
                f"Pose=({self.x:.1f}, {self.y:.1f}, {self.z:.1f}), "
                f"Yaw={math.degrees(self.yaw):.1f} deg, "
                f"-> Target=({target_x}, {target_y}, {target_z}), "
                f"dist_3D={dist_3d:.2f}, yaw_err={math.degrees(yaw_error):.1f} deg "
                f"=> cmd: vx={cmd.linear.x:.2f}, vz={cmd.linear.z:.2f}, wz={cmd.angular.z:.2f}"
            )

        # (6) Publish
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToWaypointListController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
