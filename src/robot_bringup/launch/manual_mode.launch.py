from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    robot_base_dir = get_package_share_directory('robot_base')
    robot_desc_dir = get_package_share_directory('robot_desc')
    rplidar_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch', 'rplidar.launch.py'
    )

    # Load URDF file
    urdf_path = os.path.join(robot_desc_dir, 'urdf', 'diff_robot.urdf')
    with open(urdf_path, 'r') as inf:
        robot_description = inf.read()

    return LaunchDescription([
        # Motor driver node
        Node(
            package='robot_base',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen',
        ),

        # Odometry node
        Node(
            package='robot_base',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
        ),

        # RPLiDAR node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch)
        ),

        # Robot state publisher (for URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # RViz (optional for visualization)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(robot_bringup_dir := get_package_share_directory('robot_bringup'), 'robot.rviz')]
        ),
    ])
