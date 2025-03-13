from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments for sensor hostname and lidar mode
    sensor_hostname_arg = DeclareLaunchArgument(
        'sensor_hostname', default_value='169.254.87.155',
        description='IP address of the Ouster sensor'
    )
    lidar_mode_arg = DeclareLaunchArgument(
        'lidar_mode', default_value='512x20',
        description='Lidar mode for the Ouster sensor'
    )

    # Define nodes to launch
    # ouster_driver = Node(
    #     package='ouster_ros',
    #     executable='sensor.launch.xml',
    #     name='ouster_driver',
    #     output='screen',
    #     parameters=[{
    #         'sensor_hostname': LaunchConfiguration('sensor_hostname'),
    #         'viz': False,
    #         'lidar_mode': LaunchConfiguration('lidar_mode'),
    #     }]
    # )
    ouster_driver_launch_path = os.path.join(
        get_package_share_directory('ouster_ros'),
        'launch',
        'sensor.launch.xml'
    )
    ouster_driver = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(ouster_driver_launch_path),
        launch_arguments={
            "sensor_hostname":"169.254.87.155",
            "viz":"false",
            "lidar_mode":"512x20"
        }.items()
    )

    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'pixel_format': 'mjpeg2rgb',
            'frame_rate': 60,
        }]
    )

    foxglove_bridge_launch_path = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge_launch.xml'
    )
    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(foxglove_bridge_launch_path)
    )
    imu_driver = Node(
        package='imu_driver',
        executable='imu_node',
        name='imu_driver'
    )

    gps_driver = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='gps_driver',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'baud': 9600,
        }]
    )


    # Create and return launch description
    return LaunchDescription([
        sensor_hostname_arg,
        lidar_mode_arg,
        ouster_driver,
        usb_cam,
        foxglove_bridge,
        imu_driver,
        gps_driver
    ])
