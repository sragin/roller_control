from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                [get_package_share_directory('ros2_socketcan'), '/launch/socket_can_withengine.launch.xml']
            )
        ),
        Node(
            package='gps_rclpy_pkg',
            executable='tcpgps_geoid_pub',
            parameters=[
                {'gps_ip': '192.168.150.74'},
                {'gps_port': 11512},
                {'sock_type': 'udp'}
                ]
        ),
        Node(
            package='roller_control',
            executable='roller_publisher',
        ),
    ])
