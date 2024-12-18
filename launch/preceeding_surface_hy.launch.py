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
                {'gps_ip': '192.168.48.31'},
                {'gps_port': 11511},
            ]
        ),
        Node(
            package='roller_control',
            executable='roller_publisher',
        ),
        Node(
            package='remote_control_rclpy_pkg',
            executable='remote_control_node',
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
        # Node(
        #     package='makesurface_rclpy_pkg',
        #     executable='calc_pts',
        # ),
        # Drum point 정보 - 1세부용
        # Node(
        #     package = 'makesurface_rclpy_pkg',
        #     executable = 'calcdrum_pts',
        # ),
        # 1세부 한양대 정보 보내기
        # Node(
        #     package='emulate_iotedge_pkg',
        #     executable='iotemul_roller',
        # ),

        # 관제시스템 데이터 전송 (24-09-23 : 아직 디버깅중이어서 여기서 실행하지 않음)
        Node(
            package='emulate_iotedge_pkg',
            executable='iot_roller_mqtt',
            parameters=[
                {'prj_id': '3afa400d-2ee8-4c80-8f6e-f2b8b9e55ec1'},
                {'user_id': '01012341234'},
                {'asset_id': 'a903e340-0459-4516-aacf-9307d70f5f22'},
                {'asset_num': '임090001'},
                {'asset_type': "06"},
                {'mqtt_server': 'koceti.kr'},
                {'mqtt_server_port': 1883},
                {'mqtt_id': 'koceti_client'},
                {'mqtt_pw': 'ZpfMc10fRKaJNIHR'},
                {'mqtt_topic': 'con_equipment/roller3'},
                {'logfile_duration': 1800},
                {'logfile_change_dir': 24*3600},
                {'sending_rate': 2},            # 전송 주기 (sec.)
                {'log_on': True},
            ]
        ),
    ])
