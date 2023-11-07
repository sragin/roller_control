from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                [get_package_share_directory('ros2_socketcan'), '/launch/socket_can_bridge.launch.xml']
            )
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                [get_package_share_directory('rosbridge_server'), '/launch/rosbridge_websocket_launch.xml']
            )
        ),
        Node(
            package='gps_rclpy_pkg',
            executable='tcpgps_geoid_pub',
            parameters=[
                {'gps_ip': '192.168.150.116'}
            ]
        ),
        Node(
            package='roller_control',
            executable='roller_publisher',
        ),
        # 엔진 CAN 정보를 받기 위한 것 - 1세부
        Node(
            package='engine_can_pkg',
            executable='get_engine_direct',
        ),
        # Surface 정보 생성을 위한 데이터 publish - 1,3 세부
        Node(
            package='makesurface_rclpy_pkg',
            executable='surface_pub_direct',
        ),
        # Surface 정보 생성
        Node(
            package='makesurface_rclpy_pkg',
            executable='calc_pts',
        ),
        # Drum point 정보 - 1세부용
        # Node(
        #     package = 'makesurface_rclpy_pkg',
        #     executable = 'calcdrum_pts',
        # ),
        # 1세부 한양대 정보 보내기
        Node(
            package='emulate_iotedge_pkg',
            executable='iotemul_roller',
        ),

    ])
