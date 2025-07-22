from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find paths
    robot_description_path = PathJoinSubstitution([
        FindPackageShare("robot_desc"),  # Change this to your URDF package name
        "urdf",
        "diff_robot.urdf"  # Or robot.urdf if already processed
    ])

    controller_config_path = PathJoinSubstitution([
        FindPackageShare("robot_hardware"),
        "config",
        "robot_controllers.yaml"
    ])

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description_path}, controller_config_path],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_broad"],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_cont"],
            output="screen"
        )
    ])
