from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Paths
    rplidar_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_a1_launch.py'
    )

    robot_desc_path = os.path.join(
        get_package_share_directory('robot_desc'),
        'urdf',
        'diff_robot.urdf'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('robot_bringup'),
        'robot_bringup',
        'robot.rviz'
    )

    return LaunchDescription([
        # # Launch RPLIDAR (commented out)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(rplidar_launch),
        # ),

        # Motor driver node
        Node(
            package='robot_base',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen'
        ),

        # Odometry node
        Node(
            package='robot_base',
            executable='odometry_node',
            name='odometry_node',
            output='screen'
        ),

        # Robot State Publisher (URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(robot_desc_path).read()}]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),

        # Optional: teleop node (keyboard control)
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            prefix='xterm -e',  # opens a new terminal window for keyboard input
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel')]
        )
    ])
