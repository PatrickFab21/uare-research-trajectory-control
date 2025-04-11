import rclpy
from rclpy.node import Node
import csv
import os
import time
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

class CSVSpawner(Node):
    def __init__(self):
        super().__init__('csv_spawner')
        self.declare_parameter('csv_file', '/home/patrick/ros2_ws/src/sjtu_drone_control/sjtu_drone_control/matern_type1_distribution.csv')  
        self.declare_parameter('model_name', 'person_standing')

        self.csv_file = self.get_parameter('csv_file').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        
        self.spawn_service = self.create_client(SpawnEntity, '/spawn_entity')

        self.get_logger().info(f'Esperando servicio /spawn_entity...')
        self.spawn_service.wait_for_service()
        self.get_logger().info(f'Servicio disponible. Spawneando objetos desde {self.csv_file} con modelo "{self.model_name}"...')
        
        self.spawn_objects_from_csv()

        self.get_logger().info("Todos los objetos han sido spawneados. Cerrando nodo...")
        rclpy.shutdown()

    def spawn_objects_from_csv(self):
        """ Lee el archivo CSV y spawnea los objetos en Gazebo """
        if not os.path.exists(self.csv_file):
            self.get_logger().error(f'Archivo {self.csv_file} no encontrado!')
            return
        
        with open(self.csv_file, 'r') as file:
            reader = csv.reader(file)
            header = next(reader)  # Leer el encabezado

            if header[:3] != ["Point", "X", "Y"]:
                self.get_logger().error(f'Formato de CSV incorrecto. Se esperaba: Point, X, Y')
                return
            
            object_count = 0  # Contador de objetos spawneados
            for row in reader:
                try:
                    obj_name, x, y = row
                    x, y = float(x), float(y)
                    
                    self.get_logger().info(f'Intentando spawnear: {obj_name} en ({x}, {y})')  # 🔍 VERIFICACIÓN
                    
                    self.spawn_object(obj_name, x, y)  
                    object_count += 1
                    #time.sleep(0.1)  # Esperar para evitar sobrecarga en Gazebo
                    
                except ValueError:
                    self.get_logger().error(f'Error procesando fila: {row}')

            self.get_logger().info(f'Se intentaron spawnear {object_count} objetos.')  # 🔍 VERIFICACIÓN FINAL

    def spawn_object(self, name, x, y):
        """ Spawnea un modelo específico en Gazebo en la posición X, Y """
        request = SpawnEntity.Request()
        request.name = name  
        request.xml = f"""<sdf version='1.6'>
                            <include>
                                <uri>model://{self.model_name}</uri>
                            </include>
                          </sdf>"""
        request.initial_pose = Pose()
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = 0.5  

        future = self.spawn_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)  # 🔥 Esperar a que termine el spawn
        
        if future.result() is None:
            self.get_logger().error(f'❌ Error al spawnear {name}')
        else:
            self.get_logger().info(f'✅ Spawned {name} at ({x}, {y})')


def main(args=None):
    rclpy.init(args=args)
    node = CSVSpawner()


if __name__ == '__main__':
    main()
