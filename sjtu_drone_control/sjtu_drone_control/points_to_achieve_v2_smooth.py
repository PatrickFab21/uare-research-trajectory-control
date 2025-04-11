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

        # Parámetro: número de vértices
        self.declare_parameter('num_vertices', 6)  # por defecto, hexágono
        self.num_vertices = self.get_parameter('num_vertices').get_parameter_value().integer_value

        # Parámetro: radio de giro (ajusta según ala fija o maniobrabilidad).
        self.declare_parameter('turn_radius', 15.0)
        self.turn_radius = self.get_parameter('turn_radius').get_parameter_value().double_value

        # --- Definir waypoints según número de vértices ---
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
            self.get_logger().error("Número de vértices no válido (3-7). Se usarán waypoints vacíos.")
            self.waypoints = []

        self.get_logger().info(f'Figura con {self.num_vertices} vértices y giro con radio={self.turn_radius} m: {self.waypoints}')

        # Índice del waypoint actual
        self.current_waypoint_idx = 0

        # --- Ganancias de control y tolerancias ---
        self.k_yaw = 1.0
        self.k_dist = 0.5
        self.k_z = 0.5

        self.yaw_threshold = 0.1   # rad (no se usará para frenar, solo orienta el giro)
        self.dist_threshold = 0.2  # si la distancia 3D < 0.2, consideramos alcanzado (para dron multicóptero)

        # Velocidades máximas (adáptalo para el ala fija)
        self.max_vx = 5.0
        self.max_vz = 1.0
        self.max_wz = 1.5

        # Creación de publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/simple_drone/takeoff', 10)

        # Suscripción a la Odometría
        self.odom_sub = self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)

        # Variables internas
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.has_taken_off = False

        # Timer de control (10 Hz)
        self.timer_period = 0.1
        self.control_timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info('Nodo [go_to_waypoint_list_controller] iniciado con control de giros suaves.')

    def odom_callback(self, msg: Odometry):
        """
        Lee la posición y orientación actual del dron desde la odometría y actualiza variables internas.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        self.yaw = yaw

    def control_loop(self):
        """
        Control principal para recorrer la lista de waypoints sin detenerse en los vértices.
        Utiliza un 'turn_radius' para enlazar los waypoints de forma más suave.
        """

        # 1) Despegue una sola vez
        if not self.has_taken_off or self.z < 0.5:
            self.get_logger().info("Enviando comando de despegue...")
            self.takeoff_pub.publish(Empty())
            self.has_taken_off = True
            return

        # 2) Obtener waypoint actual
        if len(self.waypoints) == 0:
            self.get_logger().warn("Sin waypoints definidos. No hay trayectoria.")
            return
        target_x, target_y, target_z = self.waypoints[self.current_waypoint_idx]

        # 3) Calcular errores
        dx = target_x - self.x
        dy = target_y - self.y
        dz = target_z - self.z

        dist_xy = math.sqrt(dx**2 + dy**2)
        dist_3d = math.sqrt(dx**2 + dy**2 + dz**2)

        # Ángulo deseado en XY
        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - self.yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))  # normaliza [-pi, pi]

        # 4) Calcular acciones de control (ya no paramos por yaw grande)
        cmd = Twist()

        # 4.1 Control yaw (sigue orientándose, pero sin frenar)
        wz = self.k_yaw * yaw_error
        wz = max(-self.max_wz, min(self.max_wz, wz))
        cmd.angular.z = wz

        # 4.2 Control de altura
        vz = self.k_z * dz
        vz = max(-self.max_vz, min(self.max_vz, vz))
        cmd.linear.z = vz

        # 4.3 Avance en X local (siempre avanza, no se detiene)
        vx = self.k_dist * dist_xy
        vx = max(-self.max_vx, min(self.max_vx, vx))
        cmd.linear.x = vx
        cmd.linear.y = 0.0  # sin desplazamiento lateral

        # 5) Condición de avance al siguiente waypoint con radio de giro
        #    Si estamos a menos de 'turn_radius' en XY, consideramos el vértice "superado"
        if dist_xy < self.turn_radius:
            self.get_logger().info(
                f"Waypoint {self.current_waypoint_idx+1} casi superado (dist={dist_xy:.2f} < radio={self.turn_radius}). "
                f"Pasando al siguiente."
            )
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)

        # (Opcional) Si queremos también detectar "final" por dist_3D < dist_threshold:
        # if dist_3d < self.dist_threshold:
        #     self.get_logger().info(
        #         f"¡Waypoint {self.current_waypoint_idx+1} alcanzado (3D)! Pasando al siguiente..."
        #     )
        #     self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)

        # 6) Publicar velocidades
        self.cmd_vel_pub.publish(cmd)

        # Imprimir info
        self.get_logger().info(
            f"Pose=({self.x:.1f},{self.y:.1f},{self.z:.1f}), "
            f"Yaw={math.degrees(self.yaw):.1f} deg, distXY={dist_xy:.2f}, yaw_err={math.degrees(yaw_error):.1f} deg "
            f"=> cmd: vx={vx:.2f}, vz={vz:.2f}, wz={wz:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoToWaypointListController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
