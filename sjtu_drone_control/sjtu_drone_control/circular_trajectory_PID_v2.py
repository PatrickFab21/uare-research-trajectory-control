#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math
import time

###############################################################################
# Helper function to convert quaternions to Euler angles (roll, pitch, yaw)
###############################################################################
def quaternion_to_euler(qx, qy, qz, qw):
    """
    0 rad in yaw => drone aligned with the global positive X-axis.
    Adjust if your simulation defines a different frame.
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


class CircularController(Node):
    """
    Node for a drone to follow a circular trajectory in XY at a certain altitude,
    with two phases: APPROACH and CIRCLE.

    Parameters (ONLY 3):
      - approach_velocity (m/s)
      - circle_velocity   (m/s)
      - circle_radius     (m)
    
    Altitude, PID gains, and approach threshold are fixed in the code.
    Publishes velocities to /cmd_vel.
    """

    def __init__(self):
        super().__init__('circular_controller')
        
        # ============== Declare parameters (only 3) ==============
        self.declare_parameter(
            'approach_velocity',
            10.0,
            descriptor=ParameterDescriptor(description="Velocity (m/s) during approach phase")
        )
        self.declare_parameter(
            'circle_velocity',
            5.817,
            descriptor=ParameterDescriptor(description="Velocity (m/s) to follow the circle")
        )
        self.declare_parameter(
            'circle_radius',
            12.036,
            descriptor=ParameterDescriptor(description="Radius of the circle (m)")
        )
        
        # ============== Load parameters ==============
        self.approach_velocity = self.get_parameter('approach_velocity').value
        self.circle_velocity   = self.get_parameter('circle_velocity').value
        self.circle_radius     = self.get_parameter('circle_radius').value
        
        # ============== Fixed settings (not parameters) ==============
        self.target_altitude    = 20.0   # Desired altitude [m]
        self.approach_threshold = 2.0    # Minimum distance to switch state
        # Fixed PID gains (XY and Z)
        self.Kp_xy = 1.0
        self.Ki_xy = 0.0
        self.Kd_xy = 0.2
        self.Kp_z = 1.0
        self.Ki_z = 0.0
        self.Kd_z = 0.2
        
        # ============== State variables ==============
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_z   = 0.0
        self.current_yaw = 0.0
        
        # Center => defined with the first odometry
        self.center_x = None
        self.center_y = None
        
        # State => APP (approach) or CIRCLE
        self.STATE_APPROACH = 1
        self.STATE_CIRCLE   = 2
        self.current_state  = self.STATE_APPROACH  # Start with approach phase
        
        # PID variables
        self.prev_err_x = 0.0
        self.sum_err_x  = 0.0
        self.prev_err_y = 0.0
        self.sum_err_y  = 0.0
        self.prev_err_z = 0.0
        self.sum_err_z  = 0.0
        
        # Time control
        self.last_time = time.time()
        
        # Internal angle for the CIRCLE phase
        self.theta = 0.0
        
        # ============== Subscriber and publisher ==============
        self.odom_sub = self.create_subscription(
            Odometry,
            '/simple_drone/odom',
            self.odom_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        
        # Timer ~10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # ============== Initial logs ==============
        self.get_logger().info("=== CircularController (two-phase) ===")
        self.get_logger().info(f"Approach velocity={self.approach_velocity} m/s, Circle velocity={self.circle_velocity} m/s")
        self.get_logger().info(f"Circle radius={self.circle_radius} m, Fixed altitude={self.target_altitude} m")

    def odom_callback(self, msg: Odometry):
        """
        On receiving odometry, update the current position.
        The first time, define (center_x, center_y) as the initial position.
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        
        q = msg.pose.pose.orientation
        _, _, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.current_yaw = yaw
        
        # Define the center the first time
        if self.center_x is None and self.center_y is None:
            self.center_x = self.current_x
            self.center_y = self.current_y
            self.get_logger().info(f"Center defined at ({self.center_x:.2f}, {self.center_y:.2f})")

    def control_loop(self):
        """ Main control loop for two phases: APPROACH and CIRCLE. Publishes /cmd_vel with PID. """
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        # Do not proceed if dt is 0 or the center is not yet defined
        if dt <= 0.0 or (self.center_x is None or self.center_y is None):
            return
        
        # 1) Calculate the desired point based on the state
        if self.current_state == self.STATE_APPROACH:
            # => target: (center_x + circle_radius, center_y)
            desired_x = self.center_x + self.circle_radius
            desired_y = self.center_y
            desired_z = self.target_altitude
            
            # Error
            err_x = desired_x - self.current_x
            err_y = desired_y - self.current_y
            err_z = desired_z - self.current_z
            
            # Horizontal distance to check if we have arrived
            dist_2d = math.sqrt(err_x**2 + err_y**2)
            
            # If close to the perimeter, switch to CIRCLE state
            if dist_2d < self.approach_threshold:
                self.get_logger().info("Approach completed => switching to CIRCLE")
                self.reset_pid_errors()
                self.current_state = self.STATE_CIRCLE
                return
            
            # NOTE: The linear velocity we "want" for approach
            # => approach_velocity. Adjust the angle 'gain' with PID,
            #    but the maximum tangential velocity is determined by the PID.
            #    Normally, we would clamp it. Here it is simplified.
            
            # Calculate PID => v_x, v_y, v_z
            vx = self.pid_control_xy(err_x, dt)
            vy = self.pid_control_xy(err_y, dt, axis='y')
            vz = self.pid_control_z(err_z, dt)
            
            # If the norm (vx, vy) is too large, scale to not exceed approach_velocity
            speed_xy = math.sqrt(vx**2 + vy**2)
            if speed_xy > self.approach_velocity:
                scale = self.approach_velocity / speed_xy
                vx *= scale
                vy *= scale
            
        else:
            # self.STATE_CIRCLE
            # Advance theta => v = w*r => w = circle_velocity / circle_radius
            w = self.circle_velocity / self.circle_radius
            self.theta += w * dt
            
            # Desired position
            desired_x = self.center_x + self.circle_radius * math.cos(self.theta)
            desired_y = self.center_y + self.circle_radius * math.sin(self.theta)
            desired_z = self.target_altitude
            
            # Error
            err_x = desired_x - self.current_x
            err_y = desired_y - self.current_y
            err_z = desired_z - self.current_z
            
            # PID
            vx = self.pid_control_xy(err_x, dt)
            vy = self.pid_control_xy(err_y, dt, axis='y')
            vz = self.pid_control_z(err_z, dt)
            
            # Restrict the magnitude (vx, vy) to circle_velocity
            speed_xy = math.sqrt(vx**2 + vy**2)
            if speed_xy > self.circle_velocity:
                scale = self.circle_velocity / speed_xy
                vx *= scale
                vy *= scale
        
        # 2) Publish velocities to /cmd_vel
        cmd_msg = Twist()
        cmd_msg.linear.x = vx
        cmd_msg.linear.y = vy
        cmd_msg.linear.z = vz
        
        self.cmd_vel_pub.publish(cmd_msg)
        
        # Log
        self.get_logger().info(
            f"[{self.current_state}] err=({err_x:.2f},{err_y:.2f},{err_z:.2f}) => "
            f"vel=({vx:.2f},{vy:.2f},{vz:.2f})"
        )

    # -------------------- PID Control for X/Y/Z -------------------- #
    def pid_control_xy(self, error, dt, axis='x'):
        """Unidimensional PID for X or Y with the same fixed gains (Kp_xy, Ki_xy, Kd_xy)."""
        if axis == 'x':
            self.sum_err_x  += error * dt
            d_err_x = (error - self.prev_err_x) / dt
            self.prev_err_x = error
            
            return (self.Kp_xy * error
                    + self.Ki_xy * self.sum_err_x
                    + self.Kd_xy * d_err_x)
        else:
            # 'y'
            self.sum_err_y  += error * dt
            d_err_y = (error - self.prev_err_y) / dt
            self.prev_err_y = error
            
            return (self.Kp_xy * error
                    + self.Ki_xy * self.sum_err_y
                    + self.Kd_xy * d_err_y)

    def pid_control_z(self, error, dt):
        """Unidimensional PID for Z with fixed Kp_z, Ki_z, Kd_z."""
        self.sum_err_z  += error * dt
        d_err_z = (error - self.prev_err_z) / dt
        self.prev_err_z = error
        
        return (self.Kp_z * error
                + self.Ki_z * self.sum_err_z
                + self.Kd_z * d_err_z)
    
    def reset_pid_errors(self):
        """Reset integrals and previous errors when switching states."""
        self.prev_err_x = 0.0
        self.sum_err_x  = 0.0
        self.prev_err_y = 0.0
        self.sum_err_y  = 0.0
        self.prev_err_z = 0.0
        self.sum_err_z  = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = CircularController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
