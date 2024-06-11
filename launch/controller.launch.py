from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=['Execute roller control launch!']),
        Node(
            package='roller_control',
            executable='base_controller',
        ),
        Node(
            package='roller_control',
            executable='roller_controller',
        ),
        Node(
            package='roller_control',
            executable='navigator',
        ),
        Node(
            package='remote_control_rclpy_pkg',
            executable='remote_control_node',
        ),
        Node(
            package='roller_control',
            executable='teleop_joystick',
        ),
    ])
