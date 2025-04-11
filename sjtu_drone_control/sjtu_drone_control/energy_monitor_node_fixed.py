import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Float32

class FixedWingEnergyModelNode(Node):
    def __init__(self):
        super().__init__('fixed_wing_energy_model_node')

        # Declare parameters for c1 and c2
        self.declare_parameter('c1', 0.075)  # Example value, adjust based on airframe, before 0.000926
        self.declare_parameter('c2', 2250.0)    # Example value, adjust based on airframe
        self.declare_parameter('mass', 7.65)    # UAV mass in kg

        # Read parameters
        self.c1 = float(self.get_parameter('c1').value)
        self.c2 = float(self.get_parameter('c2').value)
        self.m = float(self.get_parameter('mass').value)
        
        # Gravitational acceleration
        self.g = 9.81  # m/s²

        # UAV State Variables
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0

        # Publisher for power consumption
        self.power_pub = self.create_publisher(Float32, 'fixed_uav_power', 10)

        # Subscribers for velocity and acceleration
        self.vel_sub = self.create_subscription(
            Twist,
            '/simple_drone/gt_vel',
            self.vel_callback,
            10
        )
        self.acc_sub = self.create_subscription(
            Vector3,
            '/simple_drone/gt_acc',
            self.acc_callback,
            10
        )

        # Timer for periodic updates (~20 Hz)
        self.timer_ = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("FixedWingEnergyModelNode initialized.")

    def vel_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vz = msg.linear.z

    def acc_callback(self, msg: Vector3):
        self.ax = msg.x
        self.ay = msg.y
        self.az = msg.z

    def timer_callback(self):
        # Compute speed magnitude
        speed = math.sqrt(self.vx**2 + self.vy**2 + self.vz**2)
        epsilon = 1e-6  # Small value to prevent division by zero

        # Compute dot product v·a
        dot_va = self.vx * self.ax + self.vy * self.ay + self.vz * self.az

        # Compute parallel acceleration component
        if speed < epsilon:
            a_par = 0.0
            a_perp = 0.0
            induced_power = 0.0  # Avoid division by zero
        else:
            a_par = dot_va / speed
            a_perp = math.sqrt(self.ax**2 + self.ay**2 + self.az**2 - a_par**2)
            induced_power = (self.c2 / speed) * (1 + (a_perp ** 2 / self.g ** 2))

        # Compute power based on equation (68) without integration
        blade_power = self.c1 * (speed ** 3)
        tangential_power = self.m * a_par * speed
        
        # Total Power
        power = blade_power + induced_power + tangential_power

        # Publish power
        msg_power = Float32()
        msg_power.data = float(power)
        self.power_pub.publish(msg_power)

def main(args=None):
    rclpy.init(args=args)
    node = FixedWingEnergyModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
