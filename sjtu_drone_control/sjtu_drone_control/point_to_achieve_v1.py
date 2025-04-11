#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

import math

def quaternion_to_euler(qx, qy, qz, qw):
    """
    Convierte cuaternión a ángulos de Euler (roll, pitch, yaw).
    Usamos fórmulas estándar de ROS / TF:
    """
    # Referencia: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class GoTo3DPointController(Node):
    def __init__(self):
        super().__init__('go_to_3d_point_controller')

        # --- 1) Declarar parámetros para la meta 3D ---
        self.declare_parameter('target_x', 35.0)
        self.declare_parameter('target_y', 20.0)
        self.declare_parameter('target_z', 25.0)

        # --- 2) Ganancias de control y tolerancias ---
        self.k_yaw = 1.0       # Ganancia P para yaw
        self.k_dist = 0.5      # Ganancia P para avanzar en XY
        self.k_z = 0.5         # Ganancia P para altura
        self.yaw_threshold = 0.1     # rad; si el error en yaw < 0.1, comenzamos a avanzar
        self.dist_threshold = 0.2    # m  ; si la distancia 3D < 0.2, consideramos la meta lograda
        # Velocidades máximas
        self.max_vx = 2.0
        self.max_vz = 1.0
        self.max_wz = 1.5

        # --- 3) Creación de Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/simple_drone/takeoff', 10)

        # --- 4) Suscripción a la Odometría ---
        self.odom_sub = self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)

        # --- 5) Variables internas ---
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.has_taken_off = False

        # --- 6) Timer de control (10 Hz) ---
        self.timer_period = 0.1
        self.control_timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info('Nodo [go_to_3d_point_controller] iniciado.')

    def odom_callback(self, msg: Odometry):
        """
        Lee la posición y orientación actual del dron desde la odometría.
        Guarda x, y, z y yaw en variables internas.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        self.yaw = yaw  # en rad

    def control_loop(self):
        """
        Algoritmo principal de control:
         1) Asegura despegue (una sola vez).
         2) Calcula error de yaw y controla self.angular.z.
         3) Controla altura con self.linear.z (P-control).
         4) Avanza en X local cuando el yaw esté alineado.
        """

        # (1) Despegar una vez
        if not self.has_taken_off:
            self.get_logger().info("Enviando TAKEOFF...")
            self.takeoff_pub.publish(Empty())
            self.has_taken_off = True
            return

        # (2) Obtener la meta 3D
        target_x = self.get_parameter('target_x').value
        target_y = self.get_parameter('target_y').value
        target_z = self.get_parameter('target_z').value

        # (3) Calcular los errores
        dx = target_x - self.x
        dy = target_y - self.y
        dz = target_z - self.z
        dist_xy = math.sqrt(dx*dx + dy*dy)
        dist_3d = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Ángulo deseado en XY
        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - self.yaw
        # Normalizamos yaw_error a [-pi, pi]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # (4) Calcular acciones de control
        cmd = Twist()

        ## 4.1 Control de yaw
        wz = self.k_yaw * yaw_error
        # Saturamos
        if wz > self.max_wz:
            wz = self.max_wz
        elif wz < -self.max_wz:
            wz = -self.max_wz

        cmd.angular.z = wz

        ## 4.2 Control de altura
        vz = self.k_z * dz
        # Saturamos
        vz = max(-self.max_vz, min(self.max_vz, vz))
        cmd.linear.z = vz

        ## 4.3 Avance en X local
        #  - Si el error de yaw es grande, no avanzamos (para no hacer arco grande).
        #  - Si el error de yaw es pequeño, aplicamos v_x = k_dist * dist_xy, saturado.
        if abs(yaw_error) < self.yaw_threshold:
            vx = self.k_dist * dist_xy
            vx = max(-self.max_vx, min(self.max_vx, vx))
        else:
            vx = 0.0

        cmd.linear.x = vx
        cmd.linear.y = 0.0  # No nos desplazamos lateralmente

        # (5) Si estamos lo bastante cerca de la meta, detenerse.
        if dist_3d < self.dist_threshold:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("¡Meta alcanzada! Pos actual: "
                                   f"({self.x:.2f}, {self.y:.2f}, {self.z:.2f})")
            # Opcional: Podrías hacer land, o quedarte flotando.
            # self.land_pub.publish(Empty())  # si tuvieras un tópico /land
        else:
            self.get_logger().info(
                f"Pose=({self.x:.1f}, {self.y:.1f}, {self.z:.1f}), "
                f"Yaw={math.degrees(self.yaw):.1f} deg, "
                f"-> Target=({target_x}, {target_y}, {target_z}), "
                f"dist_3D={dist_3d:.2f}, yaw_err={math.degrees(yaw_error):.1f} deg "
                f"=> cmd: vx={cmd.linear.x:.2f}, vz={cmd.linear.z:.2f}, wz={cmd.angular.z:.2f}"
            )

        # (6) Publicar
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoTo3DPointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
