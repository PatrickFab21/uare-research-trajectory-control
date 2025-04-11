#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SpiralTrajectoryNode(Node):
    def __init__(self):
        super().__init__('spiral_trajectory_node')

        # Publicador para enviar comandos de velocidad
        self.cmd_vel_publisher = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        # Suscriptor de odometría para conocer la posición actual
        self.odom_subscriber = self.create_subscription(Odometry, '/simple_drone/odom', self.odom_callback, 10)

        # Parámetros de la espiral
        self.final_radius = 50.0  # Radio máximo de la espiral
        self.total_turns = 4  # Número de vueltas
        self.angular_speed = 0.5  # Velocidad angular constante (rad/s)
        self.radius_growth_rate = self.final_radius / (2 * math.pi * self.total_turns)  # Expansión del radio

        # Variables para manejar la posición inicial
        self.initial_x = None
        self.initial_y = None
        self.initial_z = None
        self.current_x = None
        self.current_y = None
        self.current_radius = 0.0
        self.theta = 0.0  # Ángulo inicial en radianes

        # Crear un temporizador para publicar comandos de velocidad
        self.timer = self.create_timer(0.1, self.publish_trajectory)

    def odom_callback(self, msg):
        """Captura la posición inicial y actualiza la posición del dron."""
        if self.initial_x is None:
            self.initial_x = msg.pose.pose.position.x
            self.initial_y = msg.pose.pose.position.y
            self.initial_z = msg.pose.pose.position.z
            self.get_logger().info(f'Posición inicial registrada: x={self.initial_x:.2f}, y={self.initial_y:.2f}, z={self.initial_z:.2f}')

        # Actualizar la posición actual
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Calcular el radio actual relativo a la posición inicial
        self.current_radius = math.sqrt((self.current_x - self.initial_x)**2 + (self.current_y - self.initial_y)**2)

    def publish_trajectory(self):
        """
        Publica un mensaje de Twist para seguir una trayectoria en espiral.
        """
        if self.initial_x is None or self.current_x is None:
            self.get_logger().info("Esperando la primera lectura de odometría...")
            return

        if self.current_radius >= self.final_radius:
            self.get_logger().info("Espiral completa, deteniendo el dron.")
            self.cmd_vel_publisher.publish(Twist())  # Detener el dron
            return

        # Incrementamos el ángulo
        self.theta += self.angular_speed * 0.1  # 0.1s es el tiempo de cada iteración

        # Calculamos el radio actual basado en el ángulo
        target_radius = self.radius_growth_rate * self.theta

        # Calculamos la velocidad lineal para expandirse
        linear_speed = target_radius * self.angular_speed

        # Creamos el mensaje Twist
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = self.angular_speed

        # Publicamos el comando de movimiento
        self.cmd_vel_publisher.publish(twist_msg)

        self.get_logger().info(f'Espiral: Ángulo={self.theta:.2f} rad, Radio objetivo={target_radius:.2f} m, '
                               f'Radio actual={self.current_radius:.2f} m, Velocidad Lineal={linear_speed:.2f} m/s')

def main(args=None):
    rclpy.init(args=args)
    node = SpiralTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
