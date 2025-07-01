from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 256000,  # A1 = 115200, A2/A3 = 256000
                'frame_id': 'rplidar_link',
                'inverted': False,
                'angle_compensate': True
            }],
            output='screen'
        )
    ])