from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Paths
    robot_desc_pkg = get_package_share_directory('robot_desc')
    bringup_pkg = get_package_share_directory('robot_bringup')
    rplidar_pkg = get_package_share_directory('rplidar_ros')

    urdf_file = os.path.join(robot_desc_pkg, 'urdf', 'diff_robot.urdf')
    rviz_file = os.path.join(bringup_pkg, 'robot_bringup', 'robot.rviz')
    controller_config = os.path.join(bringup_pkg, 'config', 'controller.yaml')
    rplidar_launch = os.path.join(rplidar_pkg, 'launch', 'rplidar_a1_launch.py')

    return LaunchDescription([

        # --- RPLIDAR (commented out) ---
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(rplidar_launch),
        # ),

        # --- Robot State Publisher ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # --- Motor Driver Node ---
        Node(
            package='robot_base',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen',
        ),

        # --- Odometry Node ---
        Node(
            package='robot_base',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
        ),

        # --- RViz2 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            output='screen'
        ),

        # --- ros2_control Controller Manager ---
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': open(urdf_file).read()},
                controller_config
            ],
            output='screen'
        ),

        # --- Controller Spawner for diff_drive_controller ---
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'diff_cont'],
            output='screen'
        ),

        # --- Teleop Twist Keyboard (for /cmd_vel) ---
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            prefix='xterm -e',  # launch in separate terminal window
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel')]
        ),
    ])

