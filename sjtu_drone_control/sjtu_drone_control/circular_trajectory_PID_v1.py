#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math
import time

###############################################################################
# Función auxiliar para convertir cuaterniones a ángulos de Euler (roll,pitch,yaw)
###############################################################################
def quaternion_to_euler(qx, qy, qz, qw):
    """
    0 rad en yaw => dron alineado con el eje X positivo global.
    Ajusta si tu simulación define otro marco.
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
    Nodo para que un dron describa una trayectoria circular en XY a cierta altitud.

    - Estado 1: (Opcional) APPROACH => acercarse al perímetro del círculo (por ej. ángulo 0).
    - Estado 2: CIRCLE => recorrer la circunferencia incrementando un ángulo interno (theta).
    
    Publica velocidades en /cmd_vel usando un PID en X, Y, Z:
      v_x = f(error_x), v_y = f(error_y), v_z = f(error_z).

    Parámetros ROS2 permiten ajustar: centro, radio, altitud, velocidad, PID, etc.
    """

    def __init__(self):
        super().__init__('circular_controller')
        
        # =============================
        # 1) Declaración de Parámetros
        # =============================
        self.declare_parameter(
            'use_odom_center',
            True,
            descriptor=ParameterDescriptor(description="Si es True, toma la primera odom como centro. Si es False, usa center_x y center_y.")
        )
        self.declare_parameter(
            'center_x',
            0.0,
            descriptor=ParameterDescriptor(description="Centro X si use_odom_center=False")
        )
        self.declare_parameter(
            'center_y',
            0.0,
            descriptor=ParameterDescriptor(description="Centro Y si use_odom_center=False")
        )
        self.declare_parameter(
            'circle_radius',
            12.036, 
            descriptor=ParameterDescriptor(description="Radio del círculo (m)")
        )
        self.declare_parameter(
            'target_altitude',
            5.817,
            descriptor=ParameterDescriptor(description="Altura deseada (m)")
        )
        self.declare_parameter(
            'linear_speed',
            5.0,
            descriptor=ParameterDescriptor(description="Velocidad tangencial deseada (m/s) para la fase CIRCLE")
        )
        self.declare_parameter(
            'two_phase_approach',
            True,
            descriptor=ParameterDescriptor(description="Si es True, primero se acerca al punto (angulo=0) antes de iniciar CIRCLE")
        )
        self.declare_parameter(
            'approach_threshold',
            2.0,
            descriptor=ParameterDescriptor(description="Distancia (m) para considerar que se alcanzó el perímetro en APPROACH")
        )
        
        # PID XY
        self.declare_parameter('Kp_xy', 1.0, descriptor=ParameterDescriptor(description='Proporcional XY'))
        self.declare_parameter('Ki_xy', 0.0, descriptor=ParameterDescriptor(description='Integral XY'))
        self.declare_parameter('Kd_xy', 0.2, descriptor=ParameterDescriptor(description='Derivativo XY'))
        
        # PID Z
        self.declare_parameter('Kp_z', 1.0, descriptor=ParameterDescriptor(description='Proporcional Z'))
        self.declare_parameter('Ki_z', 0.0, descriptor=ParameterDescriptor(description='Integral Z'))
        self.declare_parameter('Kd_z', 0.2, descriptor=ParameterDescriptor(description='Derivativo Z'))
        
        # ============================
        # 2) Carga de Parámetros
        # ============================
        self.use_odom_center   = self.get_parameter('use_odom_center').value
        self.center_x          = self.get_parameter('center_x').value
        self.center_y          = self.get_parameter('center_y').value
        self.circle_radius     = self.get_parameter('circle_radius').value
        self.target_altitude   = self.get_parameter('target_altitude').value
        self.linear_speed      = self.get_parameter('linear_speed').value
        
        self.two_phase_approach  = self.get_parameter('two_phase_approach').value
        self.approach_threshold  = self.get_parameter('approach_threshold').value
        
        # PID gains
        self.Kp_xy = self.get_parameter('Kp_xy').value
        self.Ki_xy = self.get_parameter('Ki_xy').value
        self.Kd_xy = self.get_parameter('Kd_xy').value
        
        self.Kp_z = self.get_parameter('Kp_z').value
        self.Ki_z = self.get_parameter('Ki_z').value
        self.Kd_z = self.get_parameter('Kd_z').value
        
        # ============================
        # 3) Variables de Estado
        # ============================
        # Odom actual
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_z   = 0.0
        self.current_yaw = 0.0
        
        # Centro (si use_odom_center=True, se setea en odom_callback)
        self.center_defined = (not self.use_odom_center)  # True si tenemos (center_x, center_y) listo
        
        # Ángulo interno para la fase CIRCLE
        self.theta = 0.0
        # Para la velocidad angular => v = w * r => w = v / r
        # (Si circle_radius == 0, cuidado, pero asumimos > 0)
        self.angular_speed = self.linear_speed / self.circle_radius if self.circle_radius > 0 else 0.0
        
        # Errores PID e integrales
        self.prev_err_x = 0.0
        self.sum_err_x  = 0.0
        
        self.prev_err_y = 0.0
        self.sum_err_y  = 0.0
        
        self.prev_err_z = 0.0
        self.sum_err_z  = 0.0
        
        # Control de tiempo
        self.last_time = time.time()
        
        # Estados
        self.STATE_APPROACH = 1
        self.STATE_CIRCLE   = 2
        # Iniciamos en APPROACH solo si two_phase_approach=True
        if self.two_phase_approach:
            self.current_state = self.STATE_APPROACH
        else:
            self.current_state = self.STATE_CIRCLE
        
        # ============================
        # 4) Suscriptor y Publicador
        # ============================
        self.odom_sub = self.create_subscription(
            Odometry,
            '/simple_drone/odom',
            self.odom_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        
        # Lazo de control ~10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("CircularController inicializado con parámetros:")
        self.get_logger().info(f"  use_odom_center={self.use_odom_center}, center=({self.center_x},{self.center_y}), radius={self.circle_radius}")
        self.get_logger().info(f"  target_altitude={self.target_altitude}, linear_speed={self.linear_speed}, two_phase_approach={self.two_phase_approach}")
        self.get_logger().info(f"  approach_threshold={self.approach_threshold}")
        self.get_logger().info(f"  PID XY: Kp={self.Kp_xy}, Ki={self.Ki_xy}, Kd={self.Kd_xy}")
        self.get_logger().info(f"  PID Z:  Kp={self.Kp_z},  Ki={self.Ki_z},  Kd={self.Kd_z}")

    def odom_callback(self, msg: Odometry):
        """ Actualiza la posición/orientación actual. Si se usa 'odom_center', define el centro con la 1ra odom. """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        
        q = msg.pose.pose.orientation
        _, _, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.current_yaw = yaw
        
        # Definir el centro con la primera odometría si corresponde
        if self.use_odom_center and not self.center_defined:
            self.center_x = self.current_x
            self.center_y = self.current_y
            self.center_defined = True
            self.get_logger().info(
                f"Centro definido automáticamente: ({self.center_x:.2f}, {self.center_y:.2f})"
            )

    def control_loop(self):
        """ 
        Lógica principal de control en dos estados:
          1) APPROACH => acercarse al punto en el perímetro con ángulo=0
          2) CIRCLE   => mover el dron siguiendo la circunferencia
        Luego se aplica un PID en x,y,z => publica velocidades en /cmd_vel.
        """
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        # Asegurarnos de no dividir por cero
        if dt <= 0.0:
            return
        
        if not self.center_defined:
            # Aún no tenemos centro (esperamos la 1ª odom si use_odom_center)
            return
        
        # ---------- Calcular error x,y,z según el estado ----------
        if self.current_state == self.STATE_APPROACH:
            # Objetivo => borde del círculo en ángulo 0 => (cx + r, cy)
            desired_x = self.center_x + self.circle_radius
            desired_y = self.center_y
            desired_z = self.target_altitude
            
            # Error
            err_x = desired_x - self.current_x
            err_y = desired_y - self.current_y
            err_z = desired_z - self.current_z
            
            distance = math.sqrt(err_x**2 + err_y**2)
            
            # Si estamos lo suficientemente cerca, pasamos a CIRCLE
            if distance < self.approach_threshold:
                self.get_logger().info("Aproximación completa: iniciando trayectoria circular.")
                # Reiniciamos PID e iniciamos CIRCLE
                self.reset_pid_errors()
                self.current_state = self.STATE_CIRCLE
                return
            
        elif self.current_state == self.STATE_CIRCLE:
            # Trayectoria circular => ángulo theta que aumenta
            self.theta += self.angular_speed * dt
            desired_x = self.center_x + self.circle_radius * math.cos(self.theta)
            desired_y = self.center_y + self.circle_radius * math.sin(self.theta)
            desired_z = self.target_altitude
            
            # Error
            err_x = desired_x - self.current_x
            err_y = desired_y - self.current_y
            err_z = desired_z - self.current_z
        
        else:
            # Por seguridad, si no coincide con STATE_APPROACH ni STATE_CIRCLE
            return
        
        # ---------- PID en x, y, z ----------
        vx = self.pid_control_xy(err_x, dt, axis='x')
        vy = self.pid_control_xy(err_y, dt, axis='y')
        vz = self.pid_control_z(err_z, dt)
        
        # ---------- Publicar cmd_vel ----------
        cmd_msg = Twist()
        cmd_msg.linear.x = vx
        cmd_msg.linear.y = vy
        cmd_msg.linear.z = vz
        
        # No forzamos angular.z (puedes ajustarlo si deseas cierto heading)
        cmd_msg.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd_msg)
        
        # Log para depurar
        self.get_logger().info(
            f"[{self.current_state}] err=({err_x:.2f},{err_y:.2f},{err_z:.2f}) => "
            f"cmd_vel=({vx:.2f},{vy:.2f},{vz:.2f})"
        )

    def pid_control_xy(self, error, dt, axis='x'):
        """
        PID unidimensional para eje X o Y, usando Kp_xy, Ki_xy, Kd_xy.
        """
        if axis == 'x':
            self.sum_err_x  += error * dt
            d_err_x = (error - self.prev_err_x) / dt
            self.prev_err_x = error
            
            return (self.Kp_xy * error
                    + self.Ki_xy * self.sum_err_x
                    + self.Kd_xy * d_err_x)
        else:
            # axis == 'y'
            self.sum_err_y  += error * dt
            d_err_y = (error - self.prev_err_y) / dt
            self.prev_err_y = error
            
            return (self.Kp_xy * error
                    + self.Ki_xy * self.sum_err_y
                    + self.Kd_xy * d_err_y)

    def pid_control_z(self, error, dt):
        """
        PID unidimensional para eje Z, usando Kp_z, Ki_z, Kd_z.
        """
        self.sum_err_z  += error * dt
        d_err_z = (error - self.prev_err_z) / dt
        self.prev_err_z = error
        
        return (self.Kp_z * error
                + self.Ki_z * self.sum_err_z
                + self.Kd_z * d_err_z)

    def reset_pid_errors(self):
        """ Reincia las integrales y errores previos del PID. """
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
