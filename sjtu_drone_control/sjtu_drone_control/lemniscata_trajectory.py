#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class LemniscateTrajectoryNode(Node):
    def __init__(self):
        super().__init__('lemniscate_trajectory_node')
        
        # Publicador de cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        
        # Timer para publicar la velocidad (10 Hz)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.publish_trajectory)
        
        # Parámetros de la figura en 8
        self.R = 5.0         # “Radio” o amplitud en X/Y
        self.t = 0.0         # Variable de tiempo parametrizada
        self.t_increment = 0.1  # Paso de tiempo “simulado” por callback
        
        self.get_logger().info('Nodo de trayectoria “8” iniciado.')
        
    def publish_trajectory(self):
        """
        Publica en /simple_drone/cmd_vel la velocidad lineal
        para describir una lemniscata (figura en 8) en XY,
        dejando la altura (z) y la rotación angular (angular.z) fijas.
        """
        # 1) Calculamos dx/dt, dy/dt según la derivada de la lemniscata
        dx = 2.0 * self.R * math.cos(2.0 * self.t)
        dy = self.R * math.cos(self.t)
        
        # 2) Creamos el mensaje Twist
        twist_msg = Twist()
        
        # Suponiendo que linear.x y linear.y son interpretados
        # como ejes X e Y de un marco inercial (o “world”).
        # Si tu plugin lo asume en el marco local del dron,
        # el comportamiento real puede diferir.
        
        twist_msg.linear.x = dx
        twist_msg.linear.y = dy
        
        # Mantener altura fija => no alteramos linear.z
        twist_msg.linear.z = 0.0
        
        # Tampoco giramos el dron (angular.z = 0)
        twist_msg.angular.z = 0.0
        
        # 3) Publicamos
        self.cmd_vel_publisher.publish(twist_msg)
        
        # 4) Incrementamos "tiempo" (param. de la curva)
        self.t += self.t_increment
        
        # Opcional: imprimimos una pequeña traza
        self.get_logger().info(f'Lemniscata -> dx={dx:.2f}, dy={dy:.2f}, t={self.t:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = LemniscateTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
