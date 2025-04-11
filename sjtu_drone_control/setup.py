from setuptools import setup, find_packages

package_name = 'sjtu_drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='georg.novtony@aon.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = sjtu_drone_control.teleop:main',
            'teleop_joystick = sjtu_drone_control.teleop_joystick:main',
            'open_loop_control = sjtu_drone_control.open_loop_control:main',
            'drone_position_control = sjtu_drone_control.drone_position_control:main',
            'altitude_controller = sjtu_drone_control.altitude_controller:main',
            'spiral_trajectory = sjtu_drone_control.spiral_trajectory:main',
            'circular_trajectory = sjtu_drone_control.circular_trajectory:main',
            'circular_trajectory_PID_v1 = sjtu_drone_control.circular_trajectory_PID_v1:main',
            'circular_trajectory_PID_v2 = sjtu_drone_control.circular_trajectory_PID_v2:main',
            'point_to_achieve_v1 = sjtu_drone_control.point_to_achieve_v1:main',
            'points_to_achieve_v1 = sjtu_drone_control.points_to_achieve_v1:main',
            'points_to_achieve_v2_smooth = sjtu_drone_control.points_to_achieve_v2_smooth:main',
            'points_to_achieve_v3_smooth = sjtu_drone_control.points_to_achieve_v3_smooth:main',
            'lemniscata_trajectory = sjtu_drone_control.lemniscata_trajectory:main',
            'drone_trajectory_plotter = sjtu_drone_control.drone_trajectory_plotter:main',
            'energy_monitor_node_rotary = sjtu_drone_control.energy_monitor_node_rotary:main',
            'energy_monitor_node_fixed = sjtu_drone_control.energy_monitor_node_fixed:main',
            'csv_spawner_objets = sjtu_drone_control.csv_spawner_objets:main'
        ],
    },
)