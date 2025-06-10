from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_hardware',
            executable='motor_driver',
            name='motor_driver',
            output='screen'
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -e',  # Or use 'gnome-terminal -- bash -c'
            remappings=[('/cmd_vel', '/cmd_vel')]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('robot_slam'),
                    'launch',
                    'rplidar.launch.py'
                )
            )
        )
    ])

