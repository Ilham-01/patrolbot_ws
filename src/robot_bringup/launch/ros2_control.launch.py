from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robot_description_path = os.path.join(
        get_package_share_directory('robot_desc'),
        'urdf',
        'diff_robot.urdf'
    )

    controller_config_path = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config/diffbot_controllers.yaml'
    )

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description_path, controller_config_path],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
        ),
    ])
