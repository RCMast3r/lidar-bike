import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_driver',  # Replace with the name of your first package
            executable='imu_node',   # Replace with the name of your first executable
            name='imu_node',
            output='screen',
        ),
        Node(
            package='ublox_gps',  # Replace with the name of your second package
            executable='ublox_gps_node',   # Replace with the name of your second executable
            name='ublox_gps_node',
            output='screen',
        ),
    ])
