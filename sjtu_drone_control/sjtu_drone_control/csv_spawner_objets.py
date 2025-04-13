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

        self.get_logger().info(f'Waiting for /spawn_entity service...')
        self.spawn_service.wait_for_service()
        self.get_logger().info(f'Service available. Spawning objects from {self.csv_file} using model "{self.model_name}"...')
        
        self.spawn_objects_from_csv()

        self.get_logger().info("All objects have been spawned. Shutting down node...")
        rclpy.shutdown()

    def spawn_objects_from_csv(self):
        """Reads the CSV file and spawns objects in Gazebo"""
        if not os.path.exists(self.csv_file):
            self.get_logger().error(f'File {self.csv_file} not found!')
            return
        
        with open(self.csv_file, 'r') as file:
            reader = csv.reader(file)
            header = next(reader)

            if header[:3] != ["Point", "X", "Y"]:
                self.get_logger().error(f'Invalid CSV format. Expected: Point, X, Y')
                return
            
            object_count = 0
            for row in reader:
                try:
                    obj_name, x, y = row
                    x, y = float(x), float(y)
                    
                    self.get_logger().info(f'Attempting to spawn: {obj_name} at ({x}, {y})')
                    
                    self.spawn_object(obj_name, x, y)
                    object_count += 1
                    # time.sleep(0.1)
                    
                except ValueError:
                    self.get_logger().error(f'Error processing row: {row}')

            self.get_logger().info(f'Tried to spawn {object_count} objects.')

    def spawn_object(self, name, x, y):
        """Spawns a specific model in Gazebo at position X, Y"""
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
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(f'Failed to spawn {name}')
        else:
            self.get_logger().info(f'Successfully spawned {name} at ({x}, {y})')


def main(args=None):
    rclpy.init(args=args)
    node = CSVSpawner()


if __name__ == '__main__':
    main()
