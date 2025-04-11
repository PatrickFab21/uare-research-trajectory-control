import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32

class EnergyModelNode(Node):
    def __init__(self):
        super().__init__('energy_model_node')

        # -- Declare parameters (gamma_i) with example defaults:
        self.declare_parameter('gamma_1', 504.6)   # W
        self.declare_parameter('gamma_2', 6.53e-5) # (m/s)^-2
        self.declare_parameter('gamma_3', 6.35e-3) # (m/s)^-3 * W
        self.declare_parameter('gamma_4', 687.88)  # W
        self.declare_parameter('gamma_5', 90.20)   # (m/s)^2

        # Read parameters
        self.gamma1 = float(self.get_parameter('gamma_1').value)
        self.gamma2 = float(self.get_parameter('gamma_2').value)
        self.gamma3 = float(self.get_parameter('gamma_3').value)
        self.gamma4 = float(self.get_parameter('gamma_4').value)
        self.gamma5 = float(self.get_parameter('gamma_5').value)

        # Gravity
        self.g = 9.81  # m/s^2

        # State variables
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0

        # Create publisher: instantaneous power
        self.power_pub = self.create_publisher(Float32, 'rotary_uav_power', 10)

        # Subscriptions (adjust names if your simulation uses others)
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

        # Timer (e.g., ~20 Hz)
        self.timer_ = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("EnergyModelNode ready. Using Eq.(11) from the paper for UAV power.")

    def vel_callback(self, msg: Twist):
        # Extract linear velocity
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vz = msg.linear.z
        # If angular velocity is of interest, use msg.angular.z, etc.

    def acc_callback(self, msg: Vector3):
        # Linear acceleration supposedly from IMU or derivative
        self.ax = msg.x
        self.ay = msg.y
        self.az = msg.z

    def timer_callback(self):
        # 1) Magnitude of velocity
        speed = math.sqrt(self.vx**2 + self.vy**2 + self.vz**2)

        # 2) Dot product (v·a)
        dot_va = (self.vx*self.ax + self.vy*self.ay + self.vz*self.az)

        # 3) a_par: parallel projection of a onto v
        #    a_perp = a - a_par
        #    a_perp produces the centrifugal acceleration
        # Avoid division by zero if speed ~ 0
        if speed < 1e-3:
            # Centrifugal acceleration is zero if there is no velocity
            a_c = 0.0
        else:
            # a_par
            a_par_x = (dot_va / (speed**2)) * self.vx
            a_par_y = (dot_va / (speed**2)) * self.vy
            a_par_z = (dot_va / (speed**2)) * self.vz

            a_perp_x = self.ax - a_par_x
            a_perp_y = self.ay - a_par_y
            a_perp_z = self.az - a_par_z

            a_c = math.sqrt(a_perp_x**2 + a_perp_y**2 + a_perp_z**2)

        # 4) Apply equation (11):
        # P = gamma1*(1 + gamma2 * v^2) + gamma3*(v^3)
        #   + gamma4 * sqrt(
        #         (1 + (a_c^2 / g^2))
        #         * { sqrt[ 1 + (a_c^2 / g^2) + (v^4 / gamma5^2 ) ] - v^2/gamma5 }
        #      )
        # Be careful with the notation for the nested square root
        # Adjust the "()" carefully
        # And saturate at 0 if something negative arises due to noise
        blade_power = self.gamma1*(1 + self.gamma2*(speed**2))
        parasite_power = self.gamma3*(speed**3)

        # Term for induced power
        termA = (1.0 + (a_c**2)/(self.g**2))
        inside_sqrt_B = 1.0 + (a_c**2)/(self.g**2) + (speed**4)/(self.gamma5**2)
        sqrt_B = math.sqrt(inside_sqrt_B)
        # This factor
        factor = sqrt_B - (speed**2/self.gamma5)
        if factor < 0.0:
            factor = 0.0  # Avoid square root of a negative value due to noise

        induced_part = self.gamma4 * math.sqrt(termA * factor)

        # Sum up
        power = blade_power + parasite_power + induced_part

        # Publish
        msg_power = Float32()
        msg_power.data = float(power)
        self.power_pub.publish(msg_power)


def main(args=None):
    rclpy.init(args=args)
    node = EnergyModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



