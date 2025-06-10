from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_base',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 9600
            }]
        ),
        Node(
            package='robot_base',
            executable='odometry_node',
            name='odometry_node',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 9600,
                'wheel_radius': 0.033,
                'wheel_base': 0.16,
                'ticks_per_revolution': 3200
            }]
        )
    ])
