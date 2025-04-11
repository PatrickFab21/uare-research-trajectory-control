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

        # Parámetro numérico para elegir la figura
        self.declare_parameter('num_vertices', 6)  # por defecto, hexágono
        self.num_vertices = self.get_parameter('num_vertices').get_parameter_value().integer_value

        # Definir waypoints según número de vértices
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
            self.get_logger().error("Número de vértices no válido (3-7).")
            self.waypoints = []

        self.get_logger().info(f'Figura con {self.num_vertices} vértices seleccionada: {self.waypoints}')

        # Índice del waypoint actual
        self.current_waypoint_idx = 0

        # ---------------------------------------------
        # 1) Ganancias del controlador (PID sencillo)
        # ---------------------------------------------
        self.k_yaw = 1.0   # Ganancia P para yaw
        self.k_xy  = 0.5   # Ganancia P para avanzar en XY
        self.k_z   = 0.5   # Ganancia P para altura

        # ---------------------------------------------
        # 2) Ajustes de saturación
        # ---------------------------------------------
        self.max_vx = 5.0
        self.max_vz = 1.0
        self.max_wz = 1.5

        # ---------------------------------------------
        # 3) Umbrales
        # ---------------------------------------------
        # Radio en el cual consideramos "ya pasamos la esquina"
        self.corner_radius = 5.0  # metros
        # Se elimina la lógica yaw_threshold que frenaba cuando yaw_error es grande

        # ---------------------------------------------
        # Creación de Publishers y Subscriber
        # ---------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/simple_drone/takeoff', 10)

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

        self.get_logger().info('Nodo [go_to_waypoint_list_controller] iniciado.')

    def odom_callback(self, msg: Odometry):
        """
        Lee la posición y orientación actual del dron desde la odometría.
        Actualiza x, y, z, yaw en variables internas.
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
        Control PID sencillo para seguir waypoints sin detenerse en esquinas.
        1) Despegue automático (1 sola vez).
        2) Error XY, Z, Yaw => comandos lineales/angulares saturados.
        3) Saltar al siguiente waypoint si distXY < corner_radius => giros suaves.
        """
        # 1) Despegue
        if not self.has_taken_off or self.z < 0.5:
            self.get_logger().info("Enviando comando de despegue...")
            self.takeoff_pub.publish(Empty())
            self.has_taken_off = True
            return

        # Si no hay waypoints
        if not self.waypoints:
            self.get_logger().warn("No hay waypoints definidos.")
            return

        # 2) Waypoint actual
        target_idx = self.current_waypoint_idx
        target_x, target_y, target_z = self.waypoints[target_idx]

        # 3) Calcular error
        dx = target_x - self.x
        dy = target_y - self.y
        dz = target_z - self.z

        dist_xy = math.hypot(dx, dy)
        # Nota: Podrías usar dist_3d si lo prefieres
        # dist_3d = math.sqrt(dx*dx + dy*dy + dz*dz)

        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - self.yaw
        # normalizar a [-pi, pi]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # 4) Control
        cmd = Twist()

        # 4.1 - Ajuste de yaw
        wz = self.k_yaw * yaw_error
        wz = max(-self.max_wz, min(self.max_wz, wz))
        cmd.angular.z = wz

        # 4.2 - Ajuste de altura
        vz = self.k_z * dz
        vz = max(-self.max_vz, min(self.max_vz, vz))
        cmd.linear.z = vz

        # 4.3 - Ajuste de velocidad en XY
        #     Antes se ponía vx=0 si yaw_error>threshold.
        #     Ahora siempre avanza => no se detiene en giros.
        vx = self.k_xy * dist_xy
        vx = max(0.0, min(self.max_vx, vx))  # no invertimos => solo avance
        cmd.linear.x = vx
        cmd.linear.y = 0.0

        # 5) Comprobamos si "ya pasamos" el waypoint (esquina),
        #    usando corner_radius en XY => no se acerca a 0 m,
        #    sino que cambia de waypoint a 10 m.
        if dist_xy < self.corner_radius:
            # Pasar al siguiente waypoint
            self.get_logger().info(
                f"Se alcanza zona de giro del waypoint {target_idx+1} (dist_xy={dist_xy:.2f} < {self.corner_radius}m). "
                "Pasando al siguiente..."
            )
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)
        else:
            # Opcional: Imprimir info
            self.get_logger().info(
                f"Pose=({self.x:.1f}, {self.y:.1f}, {self.z:.1f}), "
                f"YawErr={math.degrees(yaw_error):.1f} deg, "
                f"distXY={dist_xy:.2f}, -> vx={vx:.2f}, vz={vz:.2f}, wz={wz:.2f}"
            )

        # 6) Publicar
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GoToWaypointListController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
