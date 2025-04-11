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

        # Numeric parameter to select the shape
        self.declare_parameter('num_vertices', 6)  # default is hexagon
        self.num_vertices = self.get_parameter('num_vertices').get_parameter_value().integer_value

        # Define waypoints based on the number of vertices
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
            self.get_logger().error("Invalid number of vertices (3-7).")
            self.waypoints = []

        self.get_logger().info(f'Selected shape with {self.num_vertices} vertices: {self.waypoints}')

        # Current waypoint index
        self.current_waypoint_idx = 0

        # ---------------------------------------------
        # 1) Controller gains (simple PID)
        # ---------------------------------------------
        self.k_yaw = 1.0   # P gain for yaw
        self.k_xy  = 0.5   # P gain for XY movement
        self.k_z   = 0.5   # P gain for altitude

        # ---------------------------------------------
        # 2) Saturation limits
        # ---------------------------------------------
        self.max_vx = 5.0
        self.max_vz = 1.0
        self.max_wz = 1.5

        # ---------------------------------------------
        # 3) Thresholds
        # ---------------------------------------------
        # Radius to consider "corner reached"
        self.corner_radius = 5.0  # meters
        # Removed yaw_threshold logic that stopped when yaw_error was large

        # ---------------------------------------------
        # Publishers and Subscriber creation
        # ---------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/simple_drone/takeoff', 10)

        self.odom_sub = self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)

        # Internal variables
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.has_taken_off = False

        # Control timer (10 Hz)
        self.timer_period = 0.1
        self.control_timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info('Node [go_to_waypoint_list_controller] started.')

    def odom_callback(self, msg: Odometry):
        """
        Reads the drone's current position and orientation from odometry.
        Updates x, y, z, yaw in internal variables.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        _, _, yaw = quaternion_to_euler(qx, qy, qz, qw)
        self.yaw = yaw

    def control_loop(self):
        """
        Simple PID control to follow waypoints without stopping at corners.
        1) Automatic takeoff (only once).
        2) XY, Z, Yaw errors => saturated linear/angular commands.
        3) Skip to the next waypoint if distXY < corner_radius => smooth turns.
        """
        # 1) Takeoff
        if not self.has_taken_off or self.z < 0.5:
            self.get_logger().info("Sending takeoff command...")
            self.takeoff_pub.publish(Empty())
            self.has_taken_off = True
            return

        # If no waypoints are defined
        if not self.waypoints:
            self.get_logger().warn("No waypoints defined.")
            return

        # 2) Current waypoint
        target_idx = self.current_waypoint_idx
        target_x, target_y, target_z = self.waypoints[target_idx]

        # 3) Calculate error
        dx = target_x - self.x
        dy = target_y - self.y
        dz = target_z - self.z

        dist_xy = math.hypot(dx, dy)
        # Note: You could use dist_3d if preferred
        # dist_3d = math.sqrt(dx*dx + dy*dy + dz*dz)

        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - self.yaw
        # Normalize to [-pi, pi]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # 4) Control
        cmd = Twist()

        # 4.1 - Yaw adjustment
        wz = self.k_yaw * yaw_error
        wz = max(-self.max_wz, min(self.max_wz, wz))
        cmd.angular.z = wz

        # 4.2 - Altitude adjustment
        vz = self.k_z * dz
        vz = max(-self.max_vz, min(self.max_vz, vz))
        cmd.linear.z = vz

        # 4.3 - XY velocity adjustment
        #     Previously, vx=0 if yaw_error>threshold.
        #     Now it always moves forward => no stopping at turns.
        vx = self.k_xy * dist_xy
        vx = max(0.0, min(self.max_vx, vx))  # no reverse => only forward
        cmd.linear.x = vx
        cmd.linear.y = 0.0

        # 5) Check if "corner reached" using corner_radius in XY
        #    => does not approach 0 m, switches waypoint at 10 m.
        if dist_xy < self.corner_radius:
            # Move to the next waypoint
            self.get_logger().info(
                f"Reached corner zone of waypoint {target_idx+1} (dist_xy={dist_xy:.2f} < {self.corner_radius}m). "
                "Switching to the next..."
            )
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)
        else:
            # Optional: Print info
            self.get_logger().info(
                f"Pose=({self.x:.1f}, {self.y:.1f}, {self.z:.1f}), "
                f"YawErr={math.degrees(yaw_error):.1f} deg, "
                f"distXY={dist_xy:.2f}, -> vx={vx:.2f}, vz={vz:.2f}, wz={wz:.2f}"
            )

        # 6) Publish
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GoToWaypointListController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
